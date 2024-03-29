/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cw3_team_2/cw3_team_2.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

////////////////////////////////////////////////////////////////////////////////
Cw3Solution::Cw3Solution(ros::NodeHandle &nh) : g_cloud_ptr(new PointC),                            // input point cloud
                                                g_cloud_filtered(new PointC),                       // filtered point cloud
                                                g_cloud_filtered2(new PointC),                      // filtered point cloud
                                                g_cloud_plane(new PointC),                          // plane point cloud
                                                g_tree_ptr(new pcl::search::KdTree<PointT>()),      // KdTree
                                                g_cloud_normals(new pcl::PointCloud<pcl::Normal>),  // segmentation
                                                g_cloud_normals2(new pcl::PointCloud<pcl::Normal>), // segmentation
                                                g_inliers_plane(new pcl::PointIndices),             // plane seg
                                                g_coeff_plane(new pcl::ModelCoefficients),          // plane coeff
                                                debug_(false)
{
  g_nh = nh;

  // Define the publishers
  g_pub_cloud = g_nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  g_pub_pose = g_nh.advertise<geometry_msgs::PointStamped>("cube_pt", 1, true);

  // Initialize public variables
  g_vg_leaf_sz = 0.01;
  g_x_thrs_min = -0.7;
  g_x_thrs_max = -0.5;
  g_y_thrs_min = 0.0;
  g_y_thrs_max = 0.4;
  g_cf_red = 25.5;
  g_cf_blue = 204;
  g_cf_green = 25.5;
  g_k_nn = 50;

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/cw3_team_2";

  // advertise the services available from this node
  task1_srv_ = g_nh.advertiseService("/task1_start",
                                     &Cw3Solution::task1Callback, this);
  task2_srv_ = g_nh.advertiseService("/task2_start",
                                     &Cw3Solution::task2Callback, this);
  task3_srv_ = g_nh.advertiseService("/task3_start",
                                     &Cw3Solution::task3Callback, this);

  ROS_INFO("MoveIt! services initialisation finished, namespace: %s",
           service_ns.c_str());

  // Create a ROS subscriber for the input point cloud
  g_sub_cloud = g_nh.subscribe("/r200/camera/depth_registered/points", 1,
                               &Cw3Solution::cloudCallBackOne, this);
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::task1Callback(cw3_world_spawner::Task1Service::Request &request,
                                cw3_world_spawner::Task1Service::Response &response)
{
  /* This service scans the environment and finds the pose of the stack of cubes.
     After getting the stack of cubes, the colour of each of the cubes are obtained.
  */

  // clearing the list that store centroids of any previous centroid values from global variables
  clearPreviousScanData();

  int size = 0;                            // initialise variable to store number of centroids
  float yaw = 0.0;                         // initialise variable to store the orientation of the stack
  g_number_of_cubes_in_recorded_stack = 0; // initialise variable to store number of cubes in the stack
  g_check_objects_stack = true;

  // This function scans a predefined region and stores essential data from the scan in respective global variables.
  scanFrontMat();

  // FINDING ORIENTATION of the first centroid found (as only one object present in the environment)
  yaw = atan2(((clusters_max[0].x) - (clusters_max_y_x[0])), ((clusters_max[0].y) - (clusters_max_x_y[0])));

  size = centroids.size();
  g_oldcentroids = centroids;
  stack_index = 0;
  g_number_of_cubes_in_recorded_stack = g_number_of_cubes_in_stack;

  // Initializing color array and cube pixel counter, used to find an average rgb value for each cube
  for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
  {
    g_Color.r = 0.0;
    g_Color.g = 0.0;
    g_Color.b = 0.0;
    g_current_stack_colours.push_back(g_Color);
    g_current_stack_cube_color_count.push_back(0);
  }

  geometry_msgs::Pose check_col;
  check_col.position = centroids[0].point;
  check_col.position.y = check_col.position.y + 0.15;
  check_col.position.z = 0.35;

  // determine the visual check orientation
  // define placing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  q_object.setRPY(-M_PI / 4, 0, 0);
  // q_object.setRPY(-M_PI / 4, -M_PI/2 , -M_PI /4);
  q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion check_orientation;
  check_orientation = tf2::toMsg(q_result);

  check_col.orientation = check_orientation;

  moveArm(check_col);

  g_sub_cloud;

  // If a stack is found, find the average RGB values for each cube
  if (g_number_of_cubes_in_recorded_stack > 0)
  {
    for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
    {
      g_current_stack_colours[i].r = ((g_current_stack_colours[i].r) / (g_current_stack_cube_color_count[i])) / 255;
      g_current_stack_colours[i].g = ((g_current_stack_colours[i].g) / (g_current_stack_cube_color_count[i])) / 255;
      g_current_stack_colours[i].b = ((g_current_stack_colours[i].b) / (g_current_stack_cube_color_count[i])) / 255;
    }
  }
  g_check_objects_stack = false;

  // Send the pose of the stack as well as the colour of each cube to as a response service
  geometry_msgs::Point stack_point;
  stack_point.x = centroids[0].point.x;
  stack_point.y = centroids[0].point.y;
  response.stack_point = stack_point;

  response.stack_rotation = yaw;

  response.stack_colours = g_current_stack_colours;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::task2Callback(cw3_world_spawner::Task2Service::Request &request,
                                cw3_world_spawner::Task2Service::Response &response)
{

  /* This service scans the environment and stores the centroids of the cubes
      to be picked within scan area. Then gets the stack colours, and pose
      from the request. Then goes to this  where the cube is to pick and deposit it
      at the request location

  */

  // clearing the list that store centroids of any previous centroid values from global variables
  clearPreviousScanData();

  g_check_objects_floor = true;

  int size = 0; // initialise variable to store number of centroids

  scanFrontMat();

  size = centroids.size();
  g_size = size;

  // Initialise vector to store the orientation of all cubes
  for (int i = 0; i < g_size; i++)
  {
    g_yaw_list.push_back(0);
  }

  g_oldcentroids = centroids;

  for (int i = 0; i < g_size; i++)
  {
    // finding the accurate value for the centroid to the nearest second half decimal for accurate value.
    g_oldcentroids[i].point.x = floor(((g_oldcentroids[i].point.x) * 20) + 0.5) / 20;
    g_oldcentroids[i].point.y = floor(((g_oldcentroids[i].point.y) * 20) + 0.5) / 20;

    // FINDING ORIENTATION AND STORING IN LIST:
    g_yaw_list[i] = atan2(((clusters_max[i].x) - (clusters_max_y_x[i])), ((clusters_max[i].y) - (clusters_max_x_y[i])));
  }

  g_check_objects_floor = false;

  // calculating the average RGB values for each of the cubes found and approximating to the nearest 2 dp
  for (int i = 0; i < g_size; i++)
  {
    colors[i].r = ceil(((colors[i].r) / (colors_count[i])) / 255 * 10) / 10;
    colors[i].g = ceil(((colors[i].g) / (colors_count[i])) / 255 * 10) / 10;
    colors[i].b = ceil(((colors[i].b) / (colors_count[i])) / 255 * 10) / 10;
  }

  std::vector<std_msgs::ColorRGBA> list_of_colours;

  list_of_colours.clear();
  g_index_of_cubes_to_stack.clear();

  list_of_colours = request.stack_colours;

  g_num_of_cubes_to_stack = list_of_colours.size();

  // Initialise the list which will store the indices of the centroids to stack in order
  for (int i = 0; i < 3; i++)
  {
    g_index_of_cubes_to_stack.push_back(0);
  }

  // Finding the first cube which matches, using a threshold, the colour required and storing it in a vector if it has not been stored already.
  for (int i = 0; i < g_num_of_cubes_to_stack; i++)
  {
    for (int j = 0; j < size; j++)
    {
      if (abs(list_of_colours[i].r - colors[j].r) <= 0.3 && abs(list_of_colours[i].g - colors[j].g) <= 0.3 && abs(list_of_colours[i].b - colors[j].b) <= 0.3)
      {
        if (std::find(g_index_of_cubes_to_stack.begin(), g_index_of_cubes_to_stack.end(), j) != g_index_of_cubes_to_stack.end() && g_index_of_cubes_to_stack.size() > 0)
        {
          std::cout << "Element found" << std::endl;
        }
        else
        {
          g_index_of_cubes_to_stack[i] = j;
          break;
        }
      }
    }
  }

  // determine the placing location
  g_target_point.x = request.stack_point.x;
  g_target_point.y = request.stack_point.y;
  g_target_point.z = 0.03;

  // determine the placing orientation
  g_place_angle_offset_ = request.stack_rotation;

  bool success = pickAndPlaceIndexedCubes(); // Stack the cubes
  if (not success)
  {
    ROS_ERROR("Task 2 Pick and Place Failed");
    return false;
  }
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::task3Callback(cw3_world_spawner::Task3Service::Request &request,
                                cw3_world_spawner::Task3Service::Response &response)
{

  /* This service scans the entire environment and stores the centroids of the cubes
      to be picked within scan area based on the colours from a stack of cubes. Then gets the stack colours, and pose
      from the request. Then goes to this  where the cubes are to pick and stack them
      at the request location all while avoiding obstacles

  */

  // clearing the list that store centroids of any previous centroid values from global variables
  clearPreviousScanData();

  // Scan the entire mat and store the centroids present
  scanEntireMat();

  int size = centroids.size();
  g_size = size;

  // Initialise the list to store the orientation of the cubes
  for (int i = 0; i < g_size; i++)
  {
    g_yaw_list.push_back(0);
  }

  g_oldcentroids = centroids;

  for (int i = 0; i < g_size; i++)
  {

    // finding the accurate value for the centroid to the nearest second half decimal for accurate value.
    g_oldcentroids[i].point.x = floor(((g_oldcentroids[i].point.x) * 20) + 0.5) / 20;
    g_oldcentroids[i].point.y = floor(((g_oldcentroids[i].point.y) * 20) + 0.5) / 20;

    // FINDING ORIENTATION AND STORING IN LIST:
    g_yaw_list[i] = atan2(((clusters_max[i].x) - (clusters_max_y_x[i])), ((clusters_max[i].y) - (clusters_max_x_y[i])));
  }

  g_check_objects_floor = false;

  // Compute the colours of the cube and add collision objects to them.
  for (int i = 0; i < g_size; i++)
  {
    colors[i].r = ceil(((colors[i].r) / (colors_count[i])) / 255 * 10) / 10;
    colors[i].g = ceil(((colors[i].g) / (colors_count[i])) / 255 * 10) / 10;
    colors[i].b = ceil(((colors[i].b) / (colors_count[i])) / 255 * 10) / 10;

    height_vector.push_back(clusters_max[i].z);

    if (((((colors[i].r) >= 0.0) && ((colors[i].r) < 0.31)) && (((colors[i].g) >= 0.0) && ((colors[i].g) < 0.31)) && (((colors[i].b) >= 0.0) && ((colors[i].b) < 0.31))) || (std::isnan(colors[i].r)))
    {

      //////////////////////////////////////////////////////////////////////////////////
      /////// ADDING COLLISION OBJECT //////////////////////////////////////////////////

      g_collision_object = std::to_string(i);

      // this is used in defining the origin of the box collision object
      box_origin = origin(box_origin, g_oldcentroids[i].point.x, g_oldcentroids[i].point.y, g_oldcentroids[i].point.z);

      // this is used in defining the dimension of the box collision object
      box_dimension = dimension(box_dimension, 0.040, 0.040, ((clusters_max[i].z) + 0.02));

      // this is used in defining the orientation of the box collision object
      box_orientation = orientation(box_orientation, 0.0, 0.0, 0.0, 1.0);

      // function call to add a box collision object with the arguments defined above
      addCollisionObject(g_collision_object, box_origin, box_dimension, box_orientation);

      //////////////////////////////////////////////////////////////////////////////////

      g_index_of_collision_objects.push_back(i);
    }
  }

  // Removing Black cubes
  for (auto it = g_index_of_collision_objects.rbegin(); it != g_index_of_collision_objects.rend(); ++it)
  {
    colors.erase(colors.begin() + *it);
    colors_count.erase(colors_count.begin() + *it);
    g_yaw_list.erase(g_yaw_list.begin() + *it);
    g_oldcentroids.erase(g_oldcentroids.begin() + *it);
    centroids.erase(centroids.begin() + *it);
    clusters_max.erase(clusters_max.begin() + *it);
    clusters_min.erase(clusters_min.begin() + *it);
    clusters_max_y_x.erase(clusters_max_y_x.begin() + *it);
    clusters_max_x_y.erase(clusters_max_x_y.begin() + *it);
    height_vector.erase(height_vector.begin() + *it);
  }

  // Obtain the index of the centroid that contains the stack
  stack_index = std::max_element(height_vector.begin(), height_vector.end()) - height_vector.begin();

  // FINDING ORIENTATION:
  double yaw = g_yaw_list[stack_index];

  g_number_of_cubes_in_recorded_stack = round(((clusters_max[stack_index].z) - 0.017) / 0.04);

  // Initializing color array and cube pixel counter, used to find an average rgb value for each cube
  for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
  {
    g_Color.r = 0.0;
    g_Color.g = 0.0;
    g_Color.b = 0.0;
    g_current_stack_colours.push_back(g_Color);
    g_current_stack_cube_color_count.push_back(0);
  }

  geometry_msgs::Pose check_col;

  // Scan the stack of colours to determine the pose and colour of the cubes
  check_col = scan(check_col, g_oldcentroids[stack_index].point.x, g_oldcentroids[stack_index].point.y, 0.6);

  bool check_col_success = moveArm(check_col);

  g_sub_cloud;

  std::vector<std_msgs::ColorRGBA> list_of_colours;

  // If there is a stack of more than 1 cube, for every cube, compute the colour by finding the
  // average of the RGB values present in the number of pixels close to the centroid.
  if (g_number_of_cubes_in_recorded_stack > 0)
  {
    for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
    {
      g_current_stack_colours[i].r = ceil(((g_current_stack_colours[i].r) / (g_current_stack_cube_color_count[i])) / 255 * 10) / 10;
      g_current_stack_colours[i].g = ceil(((g_current_stack_colours[i].g) / (g_current_stack_cube_color_count[i])) / 255 * 10) / 10;
      g_current_stack_colours[i].b = ceil(((g_current_stack_colours[i].b) / (g_current_stack_cube_color_count[i])) / 255 * 10) / 10;
      list_of_colours.push_back(g_current_stack_colours[i]);
    }
  }
  g_check_objects_stack = false;

  //////////////////////////////////////////////////////////////////////////////////
  /////// ADDING COLLISION OBJECT //////////////////////////////////////////////////

  g_collision_object = std::to_string(stack_index);

  // this is used in defining the origin of the box collision object
  box_origin = origin(box_origin, g_oldcentroids[stack_index].point.x, g_oldcentroids[stack_index].point.y, g_oldcentroids[stack_index].point.z);

  // this is used in defining the dimension of the box collision object
  box_dimension = dimension(box_dimension, 0.040, 0.040, (height_vector[stack_index] + 0.02));

  // this is used in defining the orientation of the box collision object
  box_orientation = orientation(box_orientation, 0.0, 0.0, (g_yaw_list[stack_index] + (3.14159 / 4.0)), 1.0);

  // function call to add a box collision object with the arguments defined above
  addCollisionObject(g_collision_object, box_origin, box_dimension, box_orientation);

  //////////////////////////////////////////////////////////////////////////////////

  // Remove the stack of cubes so that the robot can only identify the singular cubes
  colors.erase(colors.begin() + stack_index);
  colors_count.erase(colors_count.begin() + stack_index);
  g_yaw_list.erase(g_yaw_list.begin() + stack_index);
  g_oldcentroids.erase(g_oldcentroids.begin() + stack_index);
  centroids.erase(centroids.begin() + stack_index);
  clusters_max.erase(clusters_max.begin() + stack_index);
  clusters_min.erase(clusters_min.begin() + stack_index);
  clusters_max_y_x.erase(clusters_max_y_x.begin() + stack_index);
  clusters_max_x_y.erase(clusters_max_x_y.begin() + stack_index);
  height_vector.erase(height_vector.begin() + stack_index);

  // this is used in defining the origin of the floor collision object
  geometry_msgs::Point floor_origin;
  floor_origin = origin(floor_origin, 0.0, 0.0, 0.0);

  // this is used in defining the dimension of the floor collision object
  geometry_msgs::Vector3 floor_dimension;
  floor_dimension = dimension(floor_dimension, 3.0, 3.0, 0.005);

  // this is used in defining the orientation of the floor collision object
  geometry_msgs::Quaternion floor_orientation;
  floor_orientation = orientation(floor_orientation, 0.0, 0.0, 0.0, 1.0);

  // function call to add a floor collision object with the arguments defined above/
  addCollisionObject("floor", floor_origin, floor_dimension, floor_orientation);

  g_index_of_cubes_to_stack.clear();

  g_num_of_cubes_to_stack = list_of_colours.size();

  // Initialising the vector which stores the indices of the cubes to be stacked
  for (int i = 0; i < g_num_of_cubes_to_stack; i++)
  {
    g_index_of_cubes_to_stack.push_back(0);
  }

  // Algorithm to find the indices of the cubes to stack
  // To find the indices, it searches through all the cubes to find the colour which matches closest to a specified colour
  // It only stores this index if it has not stored that index before
  for (int i = 0; i < g_num_of_cubes_to_stack; i++)
  {
    for (int j = 0; j < g_oldcentroids.size(); j++)
    {
      if (abs(list_of_colours[i].r - colors[j].r) <= 0.3 && abs(list_of_colours[i].g - colors[j].g) <= 0.3 && abs(list_of_colours[i].b - colors[j].b) <= 0.3)
      {
        if (std::find(g_index_of_cubes_to_stack.begin(), g_index_of_cubes_to_stack.end(), j) != g_index_of_cubes_to_stack.end() && g_index_of_cubes_to_stack.size() > 0)
        {
          std::cout << "Element found" << std::endl;
        }
        else
        {
          g_index_of_cubes_to_stack[i] = j;
          break;
        }
      }
    }
  }

  // determine the placing location
  g_target_point.x = request.stack_point.x;
  g_target_point.y = request.stack_point.y;
  g_target_point.z = 0.03;

  g_place_angle_offset_ = 0.0;

  // Function to stack the cubes
  bool success = pickAndPlaceIndexedCubes(); // Stack the cubes

  if (not success)
  {
    ROS_ERROR("Task 3 Pick and Place Failed");
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::clearPreviousScanData()
{
  // Clearing the lists that store centroids and other information of any previous detected clusters from global variables.
  centroids.clear();
  clusters_max.clear();
  clusters_min.clear();
  clusters_max_y_x.clear();
  clusters_max_x_y.clear();
  colors.clear();
  colors_count.clear();
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::scanFrontMat()
{
  // initializing variable to scan an area of the robot arm environment
  float x_scan = 0.50;
  float x_thrs_min = 0.20;
  float y_scan = 0.35;
  float y_thrs_min = 0.15;

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan1;

  for (int i = 0; i < 3; i++)
  {
    // function call setting the scan area to specific coordinate
    scan1 = scan(scan1, x_scan, y_scan, 0.7);

    // setting a threshold to store a centroid within a particular scan area
    g_x_thrs_min = x_thrs_min;
    g_y_thrs_min = y_thrs_min;
    g_x_thrs_max = g_x_thrs_min + 0.6;
    g_y_thrs_max = g_y_thrs_min + 0.3;

    // function call to move arm towards scan coordinates
    bool scan1_success = moveArm(scan1);

    // storing the centroids founds in scan area to the initialized centroids variable
    findCentroidsAtScanLocation();

    // updating the scan area for the next iteration
    y_scan -= 0.35;
    y_thrs_min -= 0.30;
  }
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::scanEntireMat()
{
  // initializing variable to scan an area of the robot arm environment
  float x_scan = 0.5;
  float x_thrs_min = 0.20;
  float y_scan = 0.35;
  float y_thrs_min = 0.15;

  g_check_objects_stack = true;
  g_check_objects_floor = true;

  g_number_of_cubes_in_recorded_stack = 0;

  // Scanning for the blue boxes at the first 3 scan locations:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan1;

  for (int i = 0; i < 3; i++)
  {
    // function call setting the scan area to specific coordinate
    scan1 = scan(scan1, x_scan, y_scan, 0.6);

    // setting a threshold to store a centroid within a particular scan area
    g_x_thrs_min = x_thrs_min;
    g_y_thrs_min = y_thrs_min;
    g_x_thrs_max = g_x_thrs_min + 0.6;
    g_y_thrs_max = g_y_thrs_min + 0.3;

    // function call to move arm towards scan coordinates
    bool scan1_success = moveArm(scan1);

    // storing the centroids founds in scan area to the initialized centroids variable
    findCentroidsAtScanLocation();

    // updating the scan area for the next iteration
    y_scan -= 0.35;
    y_thrs_min -= 0.3;
  }

  // Scanning for the blue boxes at the 4th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan4;

  // function call setting the scan area to specific coordinate
  scan4 = scan(scan4, 0.233, -0.3, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = 0.1;
  g_y_thrs_min = -0.45;
  g_x_thrs_max = g_x_thrs_min + 0.1;
  g_y_thrs_max = g_y_thrs_min + 0.3;

  // function call to move arm towards scan coordinates
  bool scan4_success = moveArm(scan4);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // Scanning for the blue boxes at the 5th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan5;

  // function call setting the scan area to specific coordinate
  scan5 = scan(scan5, -0.033, -0.3, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.1;
  g_y_thrs_min = -0.45;
  g_x_thrs_max = g_x_thrs_min + 0.2;
  g_y_thrs_max = g_y_thrs_min + 0.3;

  // function call to move arm towards scan coordinates
  bool scan5_success = moveArm(scan5);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // Scanning for the blue boxes at the 6th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan6;

  // function call setting the scan area to specific coordinate
  scan6 = scan(scan6, -0.3, -0.3, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.8;
  g_y_thrs_min = -0.45;
  g_x_thrs_max = g_x_thrs_min + 0.70;
  g_y_thrs_max = g_y_thrs_min + 0.30;

  // function call to move arm towards scan coordinates
  bool scan6_success = moveArm(scan6);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // Scanning for the blue boxes at the 7th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan7;

  // function call setting the scan area to specific coordinate
  scan7 = scan(scan7, -0.3, 0.0, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.8;
  g_y_thrs_min = -0.15;
  g_x_thrs_max = g_x_thrs_min + 0.70;
  g_y_thrs_max = g_y_thrs_min + 0.30;

  // function call to move arm towards scan coordinates
  bool scan7_success = moveArm(scan7);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // Scanning for the blue boxes at the 8th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan8;

  // function call setting the scan area to specific coordinate
  scan8 = scan(scan8, -0.3, 0.3, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.8;
  g_y_thrs_min = 0.15;
  g_x_thrs_max = g_x_thrs_min + 0.70;
  g_y_thrs_max = g_y_thrs_min + 0.30;

  // function call to move arm towards scan coordinates
  bool scan8_success = moveArm(scan8);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // Scanning for the blue boxes at the 9th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan9;

  // function call setting the scan area to specific coordinate
  scan9 = scan(scan9, -0.033, 0.3, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = -0.1;
  g_y_thrs_min = 0.15;
  g_x_thrs_max = g_x_thrs_min + 0.20;
  g_y_thrs_max = g_y_thrs_min + 0.30;

  // function call to move arm towards scan coordinates
  bool scan9_success = moveArm(scan9);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // Scanning for the blue boxes at the 10th scan location:

  // initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan10;

  // function call setting the scan area to specific coordinate
  scan10 = scan(scan10, 0.233, 0.3, 0.6);

  // setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = 0.1;
  g_y_thrs_min = 0.15;
  g_x_thrs_max = g_x_thrs_min + 0.10;
  g_y_thrs_max = g_y_thrs_min + 0.30;

  // function call to move arm towards scan coordinates
  bool scan10_success = moveArm(scan10);

  // storing the centroids founds in scan area to the initialized centroids variable
  findCentroidsAtScanLocation();

  // std::cout<< "Number of centroids found: "<<centroids.size()<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////

void Cw3Solution::findCentroidsAtScanLocation()
{
  /*this function is used to find centroids and the min and max x,y coordinates of object particular colour within
   * a particular scan area
   * When they are found, they are appended to their respective lists
   * and they are returned
   */

  geometry_msgs::PointStamped centroid;
  geometry_msgs::Point cluster_max;
  geometry_msgs::Point cluster_min;

  std_msgs::ColorRGBA color;
  int color_count;

  double cluster_max_y_x;
  double cluster_max_x_y;

  g_sub_cloud;

  int size = g_centroids.size();

  if (size > 0)
  {
    for (int i = 0; i < size; i++)
    {
      centroid = g_centroids[i];
      cluster_max = g_clusters_max[i];
      cluster_min = g_clusters_min[i];
      cluster_max_y_x = g_clusters_max_y_x[i];
      cluster_max_x_y = g_clusters_max_x_y[i];

      if (g_check_objects_floor == true)
      {
        color = g_colors[i];
        color_count = g_colors_count[i];
      }

      double x = centroid.point.x;
      double y = centroid.point.y;

      // check if the centroid found is within a particular scan area to avoid duplication
      if (((g_x_thrs_min <= x) && (x < g_x_thrs_max)) && ((g_y_thrs_min <= y) && (y < g_y_thrs_max)))
      {
        ROS_INFO("A centroid was found at this location");

        // append centroid found to the centroids list
        centroids.push_back(centroid);

        // append max centroid found to the centroids list
        clusters_max.push_back(cluster_max);
        // append min centroid found to the centroids list
        clusters_min.push_back(cluster_min);

        // append x coordinate of max y in cluster to the list
        clusters_max_y_x.push_back(cluster_max_y_x);
        // appendy coordinate of max x in cluster to the list
        clusters_max_x_y.push_back(cluster_max_x_y);

        // append color of current cluster found to the list
        colors.push_back(color);
        // append number of pixels of current cluster found to the list
        colors_count.push_back(color_count);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::pickaAndPlaceCube(std::vector<geometry_msgs::PointStamped> centroids, geometry_msgs::Point goal_loc)
{

  /*This function is used to pick and place cubes after centroid  have been found*/

  int size = centroids.size();

  // setting a condition to go through all the centroids as long as there are centroids in the list
  if (size > 0)
  {
    // looping through all the centroids
    for (int i = 0; i < size; i++)
    {
      // initializing a variable to store the coordinates of each centroid found
      geometry_msgs::Point position;
      position.x = (round(centroids[i].point.x * pow(10.0f, (2.0))) / pow(10.0f, (2.0)));
      position.y = (round(centroids[i].point.y * pow(10.0f, (2.0))) / pow(10.0f, (2.0)));
      position.z = 0.02 + (0.04 * 4);

      // function call to pick an object at the desired coordinate
      bool pick_success = pick(position);

      if (not pick_success)
      {
        ROS_ERROR("Object Pick up  failed");

        return false;
      }

      // initializing a variable to store the coordinates of the goal location(where the cube is to be placed)
      geometry_msgs::Point g_target_point;
      g_target_point.x = goal_loc.x;
      g_target_point.y = goal_loc.y;
      g_target_point.z = goal_loc.z + 0.1;

      // Function call to place the object picked inside the basket
      bool move_success = place(g_target_point);

      if (not move_success)
      {
        ROS_ERROR("Placing the object failed");

        return false;
      }
    }
  }
  return true;
}
///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose
Cw3Solution::scan(geometry_msgs::Pose scan_num, float x, float y, float z)
{
  /* This function is used to set coordinate for the desired scan area */

  // setting the values for the scan orientation
  scan_num.orientation.x = -1.0;
  scan_num.orientation.y = 0.0;
  scan_num.orientation.z = 0.0;
  scan_num.orientation.w = 0.0;

  // setting the values for the scan position
  scan_num.position.x = x;
  scan_num.position.y = y;
  scan_num.position.z = z;

  return scan_num;
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point
Cw3Solution::origin(geometry_msgs::Point collision_origin, float x, float y, float z)
{
  /* This function is used to set coordinate for the origin of collision object */

  // setting the values for the origin coordinates
  collision_origin.x = x;
  collision_origin.y = y;
  collision_origin.z = z;

  return collision_origin;
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Vector3
Cw3Solution::dimension(geometry_msgs::Vector3 collision_dimension, float x, float y, float z)
{
  /* This function is used to set coordinate for the dimension of collision object*/

  collision_dimension.x = x;
  collision_dimension.y = y;
  collision_dimension.z = z;

  return collision_dimension;
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Quaternion
Cw3Solution::orientation(geometry_msgs::Quaternion collision_orientation, float x, float y, float z, float w)
{
  /* This function is used to set coordinate for the orientation of collision object*/

  collision_orientation.x = x;
  collision_orientation.y = y;
  collision_orientation.z = z;
  collision_orientation.w = w;

  return collision_orientation;
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_)
    width = gripper_open_;
  if (width < gripper_closed_)
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);
  hand_group_.planGraspsAndPick(g_pick_object);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

void Cw3Solution::addCollisionObject(std::string object_name,
                                     geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
                                     geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::addAttachedCollisionObject(std::string object_name,
                                             geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
                                             geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::AttachedCollisionObject collision_object;
  std::vector<moveit_msgs::AttachedCollisionObject> object_vector;

  // input header information
  collision_object.object.id = object_name;
  collision_object.object.header.frame_id = base_frame_;

  collision_object.link_name = base_frame_;

  // define the primitive and its dimensions
  collision_object.object.primitives.resize(1);
  collision_object.object.primitives[0].type = collision_object.object.primitives[0].BOX;
  collision_object.object.primitives[0].dimensions.resize(3);
  collision_object.object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.object.primitive_poses.resize(1);
  collision_object.object.primitive_poses[0].position.x = centre.x;
  collision_object.object.primitive_poses[0].position.y = centre.y;
  collision_object.object.primitive_poses[0].position.z = centre.z;
  collision_object.object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object
  // hint: what about collision_object.REMOVE?
  collision_object.object.operation = collision_object.object.ADD;

  // make the collision object graspable
  collision_object.touch_links = std::vector<std::string>{"panda_hand", "panda_leftfinger", "panda_rightfinger"};

  // add the collision object to the vector, then apply to planning scene
  ROS_INFO("Adding the object into the world at the location of the hand.");
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyAttachedCollisionObjects(object_vector);

  return;
}

///////////////////////////////////////////////////////////////////////////////

void Cw3Solution::removeCollisionObject(std::string object_name)
{
  /* remove a collision object from the planning scene */

  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::pick(geometry_msgs::Point position)
{
  /* This function picks up an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define grasping as from above

  // determine the grasping orientation
  // define placing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  q_object.setRPY(0, 0, (angle_offset_ + (3.14159 / 4.0)));
  q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += z_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success)
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success)
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);

  if (not success)
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // grasp!
  success *= moveGripper(gripper_closed_);

  if (not success)
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  // retreat with object
  success *= moveArm(approach_pose);

  if (not success)
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  ROS_INFO("Pick operation successful");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::place(geometry_msgs::Point position)
{
  /* This function places an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // determine the placing orientation
  // define placing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  tf2::Quaternion q_object;
  tf2::Quaternion q_result;
  q_object.setRPY(0, 0, (angle_offset_ + (3.14159 / 4.0)));
  q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // set the desired placing pose
  geometry_msgs::Pose place_pose;
  place_pose.position = position;
  place_pose.orientation = place_orientation;
  place_pose.position.z += z_offset_;

  // set the desired pre-placing pose
  geometry_msgs::Pose approach_pose;
  approach_pose = place_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the place */

  bool success = true;

  ROS_INFO("Begining place operation");

  // move the arm above the place location
  success *= moveArm(approach_pose);

  if (not success)
  {
    ROS_ERROR("Moving arm to place approach pose failed");
    return false;
  }

  // approach to placing pose
  success *= moveArm(place_pose);

  if (not success)
  {
    ROS_ERROR("Moving arm to placing pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);
  if (not success)
  {
    ROS_ERROR("Could not open Gripper");
    return false;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool Cw3Solution::pickAndPlaceIndexedCubes()
{
  /* This function places an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  if (g_num_of_cubes_to_stack > 0)
  {

    for (int i = 0; i < g_num_of_cubes_to_stack; i++)
    {
      std::cout << "We are now trying to pick cube:  " + std::to_string(i) << std::endl;
      ;
      std::cout << "index of cube to pick: " << g_index_of_cubes_to_stack[i] << std::endl;
      std::cout << "point x of index: " << g_oldcentroids[g_index_of_cubes_to_stack[i]].point.x << std::endl;
      std::cout << "point y of index: " << g_oldcentroids[g_index_of_cubes_to_stack[i]].point.y << std::endl;
      std::cout << "point z of index: " << g_oldcentroids[g_index_of_cubes_to_stack[i]].point.z << std::endl;
      // initializing a variable to store the coordinates of each centroid found
      geometry_msgs::Point position;
      position.x = (round(g_oldcentroids[g_index_of_cubes_to_stack[i]].point.x * pow(10.0f, (2.0))) / pow(10.0f, (2.0)));
      position.y = (round(g_oldcentroids[g_index_of_cubes_to_stack[i]].point.y * pow(10.0f, (2.0))) / pow(10.0f, (2.0)));
      position.z = 0.02;

      g_pick_object = std::to_string(i);
      g_pick_objects.push_back(g_pick_object);

      angle_offset_ = g_yaw_list[g_index_of_cubes_to_stack[i]];

      // function call to pick an object from the identified coordinate
      g_pick_success = pick(position);
      if (not g_pick_success)
      {
        ROS_ERROR("Object Pick up  failed");

        return false;
      }

      // place the requested cube
      angle_offset_ = g_place_angle_offset_;
      g_place_success = place(g_target_point);
      if (not g_place_success)
      {
        ROS_ERROR("Object Placing failed");

        return false;
      }

      // open gripper before retracting arm, to avoid collision
      g_move_success = moveGripper(gripper_open_);
      if (not g_move_success)
      {
        ROS_ERROR("Opening Gripper failed");

        return false;
      }

      // determine the appropriate pose to retract arm after depositing object to avoid collision
      geometry_msgs::Pose target_pose;
      target_pose.position = g_target_point;
      target_pose.position.z = 0.3;
      // define placing as from above
      tf2::Quaternion q_x180deg(-1, 0, 0, 0);
      tf2::Quaternion q_object;
      tf2::Quaternion q_result;
      q_object.setRPY(0, 0, (angle_offset_ + (3.14159 / 4.0)));
      q_result = q_x180deg * q_object;
      target_pose.orientation = tf2::toMsg(q_result);
      // retract arm
      g_move_success = moveArm(target_pose);

      if (not g_move_success)
      {
        ROS_ERROR("Retracting arm failed");

        return false;
      }
      //////////////////////////////////////////////////////////////////////////////////
      /////// ADDING COLLISION OBJECT //////////////////////////////////////////////////

      // this is used in defining the origin of the box collision object
      box_origin = origin(box_origin, g_target_point.x, g_target_point.y, g_target_point.z);

      // this is used in defining the dimension of the box collision object
      box_dimension = dimension(box_dimension, 0.040, 0.040, 0.040);

      // this is used in defining the orientation of the box collision object
      box_orientation = orientation(box_orientation, 0.0, 0.0, g_place_angle_offset_, 1.0);

      // function call to add a box collision object with the arguments defined above
      addCollisionObject(g_pick_objects[i], box_origin, box_dimension, box_orientation);

      //////////////////////////////////////////////////////////////////////////////////

      // Incrementing target point for next deposit
      g_target_point.z = g_target_point.z + 0.04;
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
void Cw3Solution::cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

  // Perform the filtering
  applyVX(g_cloud_ptr, g_cloud_filtered);
  applyFF(g_cloud_ptr, g_cloud_filtered); // floor filtering

  // Segment plane and cube
  findNormals(g_cloud_filtered);
  segPlane(g_cloud_filtered);
  segClusters(g_cloud_filtered);

  // Clear the lists
  g_centroids.clear();
  g_clusters_max.clear();
  g_clusters_min.clear();
  g_clusters_max_y_x.clear();
  g_clusters_max_x_y.clear();
  g_colors.clear();
  g_colors_count.clear();

  std_msgs::ColorRGBA Color;

  for (std::vector<pcl::PointIndices>::const_iterator it = g_cluster_indices.begin(); it != g_cluster_indices.end(); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (const auto &idx : it->indices)
      cloud_cluster->push_back((*g_cloud_filtered)[idx]); //*
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO("Number of data points in the curent PointCloud cluster: ", cloud_cluster->size());

    // finding centroid pose of current cube cluster found
    g_current_centroid = findCubePose(cloud_cluster);

    sensor_msgs::PointCloud2 temp_cloud;
    pcl::toROSMsg(*cloud_cluster, temp_cloud);
    sensor_msgs::PointCloud cloud_cluster_camera;
    sensor_msgs::convertPointCloud2ToPointCloud(temp_cloud, cloud_cluster_camera);
    cloud_cluster_camera.header.frame_id = g_input_pc_frame_id_;
    cloud_cluster_camera.header.stamp = ros::Time(0);
    sensor_msgs::PointCloud cloud_cluster_world;
    try
    {
      g_listener_.transformPointCloud("panda_link0",
                                      cloud_cluster_camera,
                                      cloud_cluster_world);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Received a trasnformation exception: %s", ex.what());
    }

    sensor_msgs::convertPointCloudToPointCloud2(cloud_cluster_world, temp_cloud);
    PointC cloud_world;
    pcl::fromROSMsg(temp_cloud, cloud_world);

    // finding min and max depth points of the cluster
    PointT minPt, maxPt;
    pcl::getMinMax3D(cloud_world, minPt, maxPt);
    g_current_cluster_max.x = maxPt.x;
    g_current_cluster_max.y = maxPt.y;
    g_current_cluster_max.z = maxPt.z;
    g_current_cluster_min.x = minPt.x;
    g_current_cluster_min.y = minPt.y;
    g_current_cluster_min.z = minPt.z;

    g_number_of_cubes_in_stack = round(((g_current_cluster_max.z) - 0.017) / 0.04); // Calculate number of cubes in the stack

    // Initialising variables
    g_current_cluster_max_x_y = 0.0;
    g_current_cluster_max_y_x = 0.0;

    g_current_stack_colours.clear();
    g_current_stack_cube_color_count.clear();

    g_current_color.r = 0;
    g_current_color.g = 0;
    g_current_color.b = 0;
    g_current_color_count = 0;

    // Iterate through every point in a cluster
    for (int nIndex = 0; nIndex < cloud_world.size(); nIndex++)
    {

      if ((cloud_world[nIndex].x) == g_current_cluster_max.x) // Finding the y coordinate that corresoponds with the max depth points x coordiante
      {
        g_current_cluster_max_x_y = cloud_world[nIndex].y;
      }
      if ((cloud_world[nIndex].y) == g_current_cluster_max.y) // Finding the x coordinate that corresoponds with the max depth points y coordiante
      {
        g_current_cluster_max_y_x = cloud_world[nIndex].x;
      }

      if ((g_number_of_cubes_in_recorded_stack > 0) && (g_check_objects_stack == true))
      {
        // Calculating the Euclidean distance between the current centroid and the centroid of the stack
        eu_distance = sqrt(pow((g_current_centroid.point.x - g_oldcentroids[stack_index].point.x), 2) + pow((g_current_centroid.point.y - g_oldcentroids[stack_index].point.y), 2));
        if (eu_distance < 0.04)
        {

          for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
          {
            // Creating the lower bound
            g_pt_world_lb.header.frame_id = "panda_link0";
            g_pt_world_lb.header.stamp = ros::Time(0);
            g_pt_world_lb.point.x = -2;
            g_pt_world_lb.point.y = -2;
            g_pt_world_lb.point.z = ((0.03) + ((i)*0.04));

            // Creating the upper bound
            g_pt_world_ub.header.frame_id = "panda_link0";
            g_pt_world_ub.header.stamp = ros::Time(0);
            g_pt_world_ub.point.x = 2;
            g_pt_world_ub.point.y = 2;
            g_pt_world_ub.point.z = (((i + 1) * 0.04));

            if (((cloud_world[nIndex].z) < g_pt_world_ub.point.z) && ((cloud_world[nIndex].z) > g_pt_world_lb.point.z))
            { // Find the colours of the cubes on the stack by adding the RGB values of all the points in the cluster
              g_current_stack_colours[i].r = g_current_stack_colours[i].r + cloud_cluster->points[nIndex].r;
              g_current_stack_colours[i].g = g_current_stack_colours[i].g + cloud_cluster->points[nIndex].g;
              g_current_stack_colours[i].b = g_current_stack_colours[i].b + cloud_cluster->points[nIndex].b;

              g_current_stack_cube_color_count[i] = g_current_stack_cube_color_count[i] + 1;
            }
          }
        }
      }

      if (g_check_objects_floor == true) // Find the colours of the cubes on the floor by adding the RGB values of all the points in the cluster
      {

        g_current_color.r = g_current_color.r + cloud_cluster->points[nIndex].r;
        g_current_color.g = g_current_color.g + cloud_cluster->points[nIndex].g;
        g_current_color.b = g_current_color.b + cloud_cluster->points[nIndex].b;

        g_current_color_count = g_current_color_count + 1;
      }
    }

    // Store the centroids and the min and max values of the cluster to their respective clusters
    g_centroids.push_back(g_current_centroid);
    g_clusters_max.push_back(g_current_cluster_max);
    g_clusters_min.push_back(g_current_cluster_min);
    g_clusters_max_y_x.push_back(g_current_cluster_max_y_x);
    g_clusters_max_x_y.push_back(g_current_cluster_max_x_y);

    if (g_check_objects_floor == true) // Store the colours of the cubes on the floor when set to true
    {
      g_colors.push_back(g_current_color);
      g_colors_count.push_back(g_current_color_count);
    }
  }

  // Finding centroid pose of the entire filtered cloud to publish
  findCubePose(g_cloud_filtered);

  // Publish the data
  ROS_INFO("Publishing Filtered Cloud");
  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::applyVX(PointCPtr &in_cloud_ptr,
                          PointCPtr &out_cloud_ptr)
{
  /*this is used to downsample a point cloud using a voxel grid filter*/
  g_vx.setInputCloud(in_cloud_ptr);
  g_vx.setLeafSize(g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter(*out_cloud_ptr);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::applyFF(PointCPtr &in_cloud_ptr,
                          PointCPtr &out_cloud_ptr)
{

  /* This function is used to apply a colour filter to a point cloud to remove the floor*/

  geometry_msgs::PointStamped pt_camera;
  geometry_msgs::PointStamped pt_world;
  double lb_x = 0.0;
  double lb_y = 0.0;
  double lb_z = 0.03;
  pt_world.header.frame_id = "panda_link0";
  pt_world.header.stamp = ros::Time(0);
  pt_world.point.x = lb_x;
  pt_world.point.y = lb_y;
  pt_world.point.z = lb_z;
  try
  {
    g_listener_.transformPoint(g_input_pc_frame_id_,
                               pt_world,
                               pt_camera);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Received a trasnformation exception: %s", ex.what());
  }

  lb_z = pt_camera.point.z;

  // determines if a point meets this condition
  pcl::ConditionAnd<PointT>::Ptr range_condition(new pcl::ConditionAnd<PointT>());

  pcl::FieldComparison<PointT>::ConstPtr ub(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, lb_z));
  range_condition->addComparison(ub);

  g_ff.setCondition(range_condition);
  g_ff.setInputCloud(in_cloud_ptr);
  g_ff.filter(*out_cloud_ptr);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::findNormals(PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud(in_cloud_ptr);
  g_ne.setSearchMethod(g_tree_ptr);
  g_ne.setKSearch(g_k_nn);
  g_ne.compute(*g_cloud_normals);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::segPlane(PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients(true);
  g_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight(0.1);
  g_seg.setMethodType(pcl::SAC_RANSAC);
  g_seg.setMaxIterations(100);
  g_seg.setDistanceThreshold(0.03);
  g_seg.setInputCloud(in_cloud_ptr);
  g_seg.setInputNormals(g_cloud_normals);

  // Obtain the plane inliers and coefficients
  g_seg.segment(*g_inliers_plane, *g_coeff_plane);

  Cw3Solution::extractInlier(in_cloud_ptr);
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::segClusters(PointCPtr &in_cloud_ptr)
{

  /*this function is used to extract euclidean cluster*/

  std::cout << "Number of data points in the unclustered PointCloud: " << in_cloud_ptr->size() << std::endl;

  // To clear previous cluster indices
  g_cluster_indices.clear();

  g_ec.setClusterTolerance(0.02); // 2cm

  // Minimum set so that half cut cubes are not classified as clusters
  g_ec.setMinClusterSize(200);
  g_ec.setMaxClusterSize(300000);
  g_ec.setSearchMethod(g_tree_ptr);
  g_ec.setInputCloud(in_cloud_ptr);
  g_ec.extract(g_cluster_indices);
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::extractInlier(PointCPtr &in_cloud_ptr)
{
  /* A function to extract the inliers from the input cloud */

  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud(in_cloud_ptr);
  g_extract_pc.setIndices(g_inliers_plane);
  g_extract_pc.setNegative(false);

  // Write the planar inliers to disk
  g_extract_pc.filter(*g_cloud_plane);

  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative(true);
  g_extract_pc.filter(*g_cloud_filtered2);
  g_extract_normals.setNegative(true);
  g_extract_normals.setInputCloud(g_cloud_normals);
  g_extract_normals.setIndices(g_inliers_plane);
  g_extract_normals.filter(*g_cloud_normals2);
}

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::PointStamped
Cw3Solution::findCubePose(PointCPtr &in_cloud_ptr)
{

  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);

  g_cube_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cube_pt_msg.header.stamp = ros::Time(0);
  g_cube_pt_msg.point.x = centroid_in[0];
  g_cube_pt_msg.point.y = centroid_in[1];
  g_cube_pt_msg.point.z = centroid_in[2];

  // Transform the point to new frame
  geometry_msgs::PointStamped g_cube_pt_msg_out;
  try
  {
    g_listener_.transformPoint("panda_link0",
                               g_cube_pt_msg,
                               g_cube_pt_msg_out);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Received a trasnformation exception: %s", ex.what());
  }

  publishPose(g_cube_pt_msg_out);

  g_current_centroid = g_cube_pt_msg_out;

  return g_cube_pt_msg_out;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::pubFilteredPCMsg(ros::Publisher &pc_pub,
                                   PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish(g_cloud_filtered_msg);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void Cw3Solution::publishPose(geometry_msgs::PointStamped &cube_pt_msg)
{
  // Create and publish the cube pose (ignore orientation)

  g_pub_pose.publish(cube_pt_msg);

  return;
}
