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
Cw3Solution::Cw3Solution (ros::NodeHandle &nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  debug_ (false)
{
  g_nh = nh;

  // Define the publishers
  g_pub_cloud = g_nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_pose = g_nh.advertise<geometry_msgs::PointStamped> ("cube_pt", 1, true);
  
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
  g_sub_cloud = g_nh.subscribe("/r200/camera/depth_registered/points",1,
    &Cw3Solution::cloudCallBackOne,this);
}


///////////////////////////////////////////////////////////////////////////////

bool
Cw3Solution::task1Callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response)
{
  /* This service picks an object with a given pose and places it at a given pose */


  // clearing the list that store centroids of any previous centroid values
  centroids.clear();
  centroids_max.clear();
  centroids_min.clear();
  centroids_max_depth.clear();
  centroids_min_depth.clear();

  int size = 0;
  float yaw = 0.0;
  std_msgs::ColorRGBA Color;
  g_number_of_cubes_in_recorded_stack = 0;


  //initializing variable to scan an area of the robot arm environment
  float x_scan = 0.50;
  float x_thrs_min = 0.20;
  float y_scan = 0.35;
  float y_thrs_min = 0.15;



  //initializing a variable to scan an area of the robot arm environment
  geometry_msgs::Pose scan1;

  for (int i = 0; i < 3; i++)
  {
    //function call setting the scan area to specific coordinate
    scan1 = scan(scan1, x_scan, y_scan, 0.6);

    //setting a threshold to store a centroid within a particular scan area
    g_x_thrs_min = x_thrs_min;
    g_y_thrs_min = y_thrs_min;
    g_x_thrs_max = g_x_thrs_min + 0.6;
    g_y_thrs_max = g_y_thrs_min + 0.3;
    
    //function call to move arm towards scan coordinates
    bool scan1_success = moveArm(scan1);
    
    //storing the centroids founds in scan area to the initialized centroids variable (Change comment ...)
    findCentroidsAtScanLocation();
    
    
    //updating the scan area for the next iteration
    y_scan -= 0.35;
    y_thrs_min -= 0.30;
  }

  //function call setting the scan area to coordinate where the stack was found
  scan1 = scan(scan1, centroids[0].point.x, centroids[0].point.y, 0.6);
  
  //function call to move arm towards scan coordinates
  bool scan1_success = moveArm(scan1);

  // clearing the list that store centroids of any previous centroid values
  centroids.clear();
  centroids_max.clear();
  centroids_min.clear();
  centroids_max_depth.clear();
  centroids_min_depth.clear();

  //setting a threshold to store a centroid within a particular scan area
  g_x_thrs_min = centroids[0].point.x - 0.4;
  g_y_thrs_min = centroids[0].point.y - 0.4;
  g_x_thrs_max = centroids[0].point.x + 0.4;
  g_y_thrs_max = centroids[0].point.y + 0.4;
  
  //storing the centroids founds in scan area to the initialized centroids variable (Change comment ...)
  findCentroidsAtScanLocation();

  size = centroids.size();
  g_number_of_cubes_in_recorded_stack = g_number_of_cubes_in_stack;

  // Initializing color array
  for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
  {
    Color.r = 0.0;
    Color.g = 0.0;
    Color.b = 0.0;
    g_current_stack_colours.push_back(Color);
  }
  
  if (size > 0)
  {
      for (int i = 0; i < size; i++)
      {
        std::cout << "This is centroid " + char(i)  << std::endl;
        std::cout << centroids[i]  << std::endl;

        std::cout << "The max x, y, z value of this cluster is: "  << std::endl;
        std::cout << centroids_max[i]  << std::endl;
        std::cout << "The min x, y, z value of this cluster is: "  << std::endl;
        std::cout << centroids_min[i]  << std::endl;

        std::cout << "The y value for the max x of this cluster is: "  << std::endl;
        std::cout << g_current_centroid_max_x_y  << std::endl;
        std::cout << "The x value for the max y of this cluster is: "  << std::endl;
        std::cout << g_current_centroid_max_y_x  << std::endl;
        

        std::cout << "The max depth of this centroid is: "  << std::endl;
        std::cout << centroids_max_depth[i]  << std::endl;
        std::cout << "The min depth of this centroid is: "  << std::endl;
        std::cout << centroids_min_depth[i]  << std::endl;

        std::cout << "The number of cubes in this stack is: "  << std::endl;
        std::cout << g_number_of_cubes_in_recorded_stack  << std::endl;
      }
  }

  // FINDING ORIENTATION:
  yaw = atan2(((centroids_max[0].x) - (g_current_centroid_max_y_x)),((centroids_max[0].y) - (g_current_centroid_max_x_y)));

  std::cout << "The angle is: "  << std::endl;
  std::cout << yaw  << std::endl;



  geometry_msgs::Pose check_col;
  check_col.position = centroids[0].point;
  check_col.position.y = check_col.position.y + 0.3;
  check_col.position.z = 0.03 + ((0.04*g_number_of_cubes_in_recorded_stack)/2);

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the placing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(-M_PI / 4, -M_PI/2 , -M_PI /4);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);
  
  check_col.orientation = orientation;




  moveArm(check_col);

  // geometry_msgs::Point goal_loc;
  // goal_loc.x = 0.5;
  // goal_loc.y = 0.5;
  // goal_loc.z = 0;
  // bool pickCubes = pickaAndPlaceCube(centroids, goal_loc);


  //storing the centroids founds in scan area to the initialized centroids variable (Change comment ...)
  g_sub_cloud;

  //size = g_current_stack_colours.size();
  
  if (g_number_of_cubes_in_recorded_stack > 0)
  {
    for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
    {
      //subscribing to the cloud to check for colour
      g_sub_cloud;
      std::cout << "This is the colour for cube: " << i << std::endl;
      std::cout << "Colour: "  << std::endl;
      std::cout << g_current_stack_colours[i]  << std::endl;
    }
  }


}


///////////////////////////////////////////////////////////////////////////////

bool
Cw3Solution::task2Callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  
  /* This service .
  
  */
  


  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
Cw3Solution::task3Callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{

  /* This service ...
  */
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
Cw3Solution::findCentroidsAtScanLocation()
{
  /*this function is used to find centroids of object particular colour within
    * a particular scan area
    * If centroid is found, it is appended to the centroids list created
    * and this is then returned
   */
  /* Change function name and description, also in header, as even min and max values are found now, also i do not parse anything inside anymore ...*/
  

  geometry_msgs::PointStamped centroid;
  geometry_msgs::Point centroid_max;
  geometry_msgs::Point centroid_min;
  double centroid_max_depth;
  double centroid_min_depth;

  g_sub_cloud;

  int size = g_centroids.size();

  if (size > 0)
  {
    for (int i = 0; i < size; i++)
    {
      centroid = g_centroids[i];
      centroid_max = g_centroids_max[i];
      centroid_min = g_centroids_min[i];
      // centroid_max_depth = g_centroids_max_depth[i];
      // centroid_min_depth = g_centroids_min_depth[i];

      double x = centroid.point.x;
      double y = centroid.point.y;

      //check if the centroid found is within a particular scan area to avoid duplication
      if (((g_x_thrs_min <= x ) && (x < g_x_thrs_max)) && ((g_y_thrs_min <= y ) && (y < g_y_thrs_max)))
      {
        ROS_INFO("A centroid was found at this location");

        //append centroid found to the centroids list
        centroids.push_back(centroid);
        //append max centroid found to the centroids list
        centroids_max.push_back(centroid_max);
        //append max centroid found to the centroids list
        centroids_min.push_back(centroid_min);
        //append cluster's max depth found to the centroids list
        centroids_max_depth.push_back(centroid_max_depth);
        //append cluster's min depth found to the centroids list
        centroids_min_depth.push_back(centroid_min_depth);
      }
    }

  }
  

}


///////////////////////////////////////////////////////////////////////////////

bool
Cw3Solution::pickaAndPlaceCube(std::vector<geometry_msgs::PointStamped> centroids, geometry_msgs::Point goal_loc)
{

  /*This function is used to pick and place cubes after centroid  have been found*/


   int size = centroids.size();

  //setting a condition to go through all the centroids as long as there are centroids in the list
  if (size > 0)
  {
      //looping through all the centroids
      for (int i = 0; i < size; i++)
      {
        std::cout << "We are now trying to pick cube:  " + std::to_string(i)  << std::endl;
        
        //initializing a variable to store the coordinates of each centroid found
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
        geometry_msgs::Point target_point;
        target_point.x = goal_loc.x;
        target_point.y = goal_loc.y;
        target_point.z = goal_loc.z+0.1;

        // Function call to place the object picked inside the basket
        bool move_success = place(target_point);

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
  
  //setting the values for the scan orientation
  scan_num.orientation.x = -1.0;
  scan_num.orientation.y = 0.0;
  scan_num.orientation.z = 0.0;
  scan_num.orientation.w = 0.0;

  //setting the values for the scan position
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
  
  //setting the values for the origin coordinates
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
void
Cw3Solution::addFloorAndBoxCollisionObjects(geometry_msgs::PointStamped goal_loc)
{

  // this is used in defining the origin of the floor collision object
  geometry_msgs::Point floor_origin;
  floor_origin = origin(floor_origin, 0.0, 0.0, 0.0);

  // this is used in defining the dimension of the floor collision object
  geometry_msgs::Vector3 floor_dimension;
  floor_dimension = dimension(floor_dimension, 3.0, 3.0, 0.001);

  // this is used in defining the orientation of the floor collision object
  geometry_msgs::Quaternion floor_orientation;
  floor_orientation = orientation(floor_orientation, 0.0, 0.0,0.0,1.0);


  // function call to add a floor collision object with the arguments defined above/
  addCollisionObject("cube_1",floor_origin,floor_dimension,floor_orientation);


  // this is used in defining the origin of box collision object
  geometry_msgs::Point box_origin;
  box_origin = origin(box_origin, goal_loc.point.x, goal_loc.point.y, 0.0);


  // this is used in defining the dimension of the box collision object
  geometry_msgs::Vector3 box_dimension;
  box_dimension = dimension(box_dimension, 0.22, 0.22, 0.45);


  // this is used in defining the orientation of the box collision object
  geometry_msgs::Quaternion box_orientation;
  box_orientation = orientation(box_orientation, 0.0, 0.0,0.0,1.0);


  // function call to add a box collision object with the arguments defined above
  addCollisionObject("cube_2",box_origin,box_dimension,box_orientation);

}

///////////////////////////////////////////////////////////////////////////////

bool
Cw3Solution::moveArm(geometry_msgs::Pose target_pose)
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

bool
Cw3Solution::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

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

void
Cw3Solution::addCollisionObject(std::string object_name,
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

///////////////////////////////////////////////////////////////////////////////

void
Cw3Solution::removeCollisionObject(std::string object_name)
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

bool
Cw3Solution::pick(geometry_msgs::Point position)
{
  /* This function picks up an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
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

bool
Cw3Solution::place(geometry_msgs::Point position)
{
  /* This function places an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define placing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the placing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
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

void
Cw3Solution::cloudCallBackOne
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);


  // Perform the filtering
  applyVX (g_cloud_ptr, g_cloud_filtered);
  //applyPT (g_cloud_ptr, g_cloud_filtered);
  //applyBCF (g_cloud_ptr, g_cloud_filtered);
  applyFF (g_cloud_ptr, g_cloud_filtered);
  //applyCF (g_cloud_ptr, g_cloud_filtered);
  
  // Segment plane and cube
  findNormals (g_cloud_filtered);
  segPlane (g_cloud_filtered);
  segClusters (g_cloud_filtered);

  g_centroids.clear();
  g_centroids_max.clear();
  g_centroids_min.clear();
  // g_centroids_max_depth.clear();
  // g_centroids_min_depth.clear();



  for (std::vector<pcl::PointIndices>::const_iterator it = g_cluster_indices.begin (); it != g_cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (const auto& idx : it->indices)
    cloud_cluster->push_back ((*g_cloud_filtered)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_INFO("Number of data points in the curent PointCloud cluster: ", cloud_cluster->size () );

    // finding centroid pose of current cube cluster found
    g_current_centroid = findCubePose (cloud_cluster);

    sensor_msgs::PointCloud2 temp_cloud;
    pcl::toROSMsg(*cloud_cluster, temp_cloud);
    sensor_msgs::PointCloud cloud_cluster_camera;
    sensor_msgs::convertPointCloud2ToPointCloud(temp_cloud, cloud_cluster_camera);
    cloud_cluster_camera.header.frame_id = g_input_pc_frame_id_;
    cloud_cluster_camera.header.stamp = ros::Time (0);
    sensor_msgs::PointCloud cloud_cluster_world;
    try
    {
      g_listener_.transformPointCloud ("panda_link0",
                                  cloud_cluster_camera,
                                  cloud_cluster_world);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
    }

    sensor_msgs::convertPointCloudToPointCloud2(cloud_cluster_world, temp_cloud);
    PointC cloud_world;
    pcl::fromROSMsg(temp_cloud, cloud_world);

    //finding min and max depth points of the cluster
    PointT minPt, maxPt;
    pcl::getMinMax3D (cloud_world, minPt, maxPt);

    g_current_centroid_max.x = maxPt.x;
    g_current_centroid_max.y = maxPt.y;
    g_current_centroid_max.z = maxPt.z;

    g_current_centroid_min.x = minPt.x;
    g_current_centroid_min.y = minPt.y;
    g_current_centroid_min.z = minPt.z;

    g_number_of_cubes_in_stack = round(((g_current_centroid_max.z)-0.03)/0.04);

    g_current_centroid_max_x_y = 0.0;
    g_current_centroid_max_y_x = 0.0;

    //g_current_stack_colours.clear();

    for(int nIndex = 0; nIndex < cloud_world.size (); nIndex++)
    {

      if ((cloud_world[nIndex].x) == g_current_centroid_max.x)
      {
        g_current_centroid_max_x_y = cloud_world[nIndex].y;
      }
      if ((cloud_world[nIndex].y) == g_current_centroid_max.y)
      {
        g_current_centroid_max_y_x = cloud_world[nIndex].x;
      }

      if (g_number_of_cubes_in_recorded_stack > 0)
      {
        for (int i = 0; i < g_number_of_cubes_in_recorded_stack; i++)
        {

          g_pt_world_lb.header.frame_id = "panda_link0";
          g_pt_world_lb.header.stamp = ros::Time (0);
          g_pt_world_lb.point.x = -20;
          g_pt_world_lb.point.y = -20;
          g_pt_world_lb.point.z = ((0.03)+((i-1)*0.04/2));
          try
          {
            g_listener_.transformPoint (g_input_pc_frame_id_,
                                        g_pt_world_lb,
                                        g_pt_camera_lb);
                                        
          }
          catch (tf::TransformException& ex)
          {
            ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
          }

          g_pt_world_ub.header.frame_id = "panda_link0";
          g_pt_world_ub.header.stamp = ros::Time (0);
          g_pt_world_ub.point.x = 20;
          g_pt_world_ub.point.y = 20;
          g_pt_world_ub.point.z = ((0.03)+(i*0.04/2));
          try
          {
            g_listener_.transformPoint (g_input_pc_frame_id_,
                                        g_pt_world_ub,
                                        g_pt_camera_ub);
                                        
          }
          catch (tf::TransformException& ex)
          {
            ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
          }

          
          // if ((((cloud_cluster->points[nIndex].z) < g_pt_camera_ub.point.z) && ((cloud_cluster->points[nIndex].z) >  g_pt_camera_lb.point.z))
          //     && (((cloud_cluster->points[nIndex].y) < g_pt_camera_ub.point.y) && ((cloud_cluster->points[nIndex].y) >  g_pt_camera_lb.point.y))
          //     && (((cloud_cluster->points[nIndex].x) < g_pt_camera_ub.point.x) && ((cloud_cluster->points[nIndex].x) >  g_pt_camera_lb.point.x)))
          // {
          //   g_Color.r = cloud_cluster->points[nIndex].r;
          //   g_Color.g = cloud_cluster->points[nIndex].g;
          //   g_Color.b = cloud_cluster->points[nIndex].b;
          //   g_current_stack_colours[i] = g_Color;
          //   ROS_ERROR("IT COMES HERE! ...");
          // }

          if (((cloud_world[nIndex].z) < g_pt_world_ub.point.z)&&((cloud_world[nIndex].z) > g_pt_world_lb.point.z))
          {
            g_Color.r = cloud_cluster->points[nIndex].r;
            g_Color.g = cloud_cluster->points[nIndex].g;
            g_Color.b = cloud_cluster->points[nIndex].b;
            g_current_stack_colours[i] = g_Color;
            ROS_ERROR("IT COMES");
          }

        }
      }
    }

    // g_current_centroid_max_depth = maxPt.z;
    // g_current_centroid_min_depth = minPt.z;




    g_centroids.push_back(g_current_centroid);
    g_centroids_max.push_back(g_current_centroid_max);
    g_centroids_min.push_back(g_current_centroid_min);
    // g_centroids_max_depth.push_back(g_current_centroid_max_depth);
    // g_centroids_min_depth.push_back(g_current_centroid_min_depth);

  }
  // Finding centroid pose of the entire filtered cloud to publish
  findCubePose (g_cloud_filtered);
    
  // Publish the data
  ROS_INFO ("Publishing Filtered Cloud");
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);
  //g_pub_cloud.publish(g_cloud_normals2);
  //pubFilteredPCMsg (g_pub_cloud, *g_cloud_cylinder);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::applyVX (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  /*this is used to downsample a point cloud using a voxel grid filter*/
  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::applyPT (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  /*this is used to ...*/
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits (0.03, 1000); //put in config ...
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::applyCF (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{


  /* This function is used to apply a colour filter to a point cloud*/


  //determines if a point meets this condition
  pcl::ConditionAnd<PointT>::Ptr condition (new pcl::ConditionAnd<PointT> ());


  // set the Lower bound for red conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_red(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::GT, g_cf_red-25.5));
  condition->addComparison (lb_red);

  // set the Lower bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_green(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::GT, g_cf_green-25.5));
  condition->addComparison (lb_green);

  // set the Lower bound for blue conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_blue(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::GT, g_cf_blue-25.5));
  condition->addComparison (lb_blue);

 
  // set the upper bound for red conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_red(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::LT, g_cf_red+25.5));
  condition->addComparison (ub_red);

  // set the upper bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_green(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LT, g_cf_green+25.5));
  condition->addComparison (ub_green);

  // set the upper bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_blue(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::LT, g_cf_blue+25.5));
  condition->addComparison (ub_blue);

  g_cf.setInputCloud (in_cloud_ptr);
  g_cf.setCondition (condition);
  g_cf.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::applyBCF (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{


  /* This function is used to apply a colour filter to a point cloud to remove black color*/
  /*Need to put the 0.1 inside a config file*/

  //determines if a point meets this condition
  pcl::ConditionAnd<PointT>::Ptr condition (new pcl::ConditionAnd<PointT> ());

  // set the Lower bound for red conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_red(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::GT, (0.1*255)-25.5));
  condition->addComparison (lb_red);

  // set the Lower bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_green(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::GT, (0.1*255)-25.5));
  condition->addComparison (lb_green);

  // set the Lower bound for blue conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr lb_blue(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::GT, (0.1*255)-25.5));
  condition->addComparison (lb_blue);

 
 // set the upper bound for red conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_red(new pcl::PackedRGBComparison<PointT>("r", pcl::ComparisonOps::LT, (0.1*255)+25.5));
  condition->addComparison (ub_red);

  // set the upper bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_green(new pcl::PackedRGBComparison<PointT>("g", pcl::ComparisonOps::LT, (0.1*255)+25.5));
  condition->addComparison (ub_green);

  // set the upper bound for green conditional colour filter
  pcl::PackedRGBComparison<PointT>::ConstPtr ub_blue(new pcl::PackedRGBComparison<PointT>("b", pcl::ComparisonOps::LT, (0.1*255)+25.5));
  condition->addComparison (ub_blue);

  g_cf.setInputCloud (in_cloud_ptr);
  g_cf.setCondition (condition);
  g_cf.filter (*out_cloud_ptr);
  
  return;
}


////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::applyFF (PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{


  /* This function is used to apply a colour filter to a point cloud to remove the floor*/
  /* Remove g_fseg later if not needed ...*/

  // // Optional
  // g_fseg.setOptimizeCoefficients (true);
  // // Mandatory
  // g_fseg.setModelType (pcl::SACMODEL_PLANE);
  // g_fseg.setMethodType (pcl::SAC_RANSAC);
  // g_fseg.setDistanceThreshold (0.01);

  // g_fseg.setInputCloud (in_cloud_ptr);
  // g_fseg.segment (*g_inliers_plane, *g_coeff_plane);

  //change ...
  geometry_msgs::PointStamped pt_camera;
  geometry_msgs::PointStamped pt_world;
  double lb_x = 0.0;
  double lb_y = 0.0;
  double lb_z = 0.04;
  pt_world.header.frame_id = "panda_link0";
  pt_world.header.stamp = ros::Time (0);
  pt_world.point.x = lb_x;
  pt_world.point.y = lb_y;
  pt_world.point.z = lb_z;
  try
  {
    g_listener_.transformPoint (g_input_pc_frame_id_,
                                pt_world,
                                pt_camera);
                                
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }
  
  lb_z = pt_camera.point.z;


  //determines if a point meets this condition
  pcl::ConditionAnd<PointT>::Ptr range_condition (new pcl::ConditionAnd<PointT> ());

  pcl::FieldComparison<PointT>::ConstPtr ub(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, lb_z));
  range_condition->addComparison (ub);

  // pcl::FieldComparison<PointT>::ConstPtr lb(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 100));
  // range_condition->addComparison (lb);
  // build the filter

  g_ff.setCondition (range_condition);
  g_ff.setInputCloud (in_cloud_ptr);
  //g_ff.setKeepOrganized(true);
  g_ff.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight (0.1);
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); 
  g_seg.setDistanceThreshold (0.03); 
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);

  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);


  Cw3Solution::extractInlier (in_cloud_ptr);

}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::segClusters (PointCPtr &in_cloud_ptr)
{

  /*this function is used to extract euclidean cluster*/

  std::cout << "Number of data points in the unclustered PointCloud: " << in_cloud_ptr->size () << std::endl;

  //To clear previous cluster indices
  g_cluster_indices.clear();

  g_ec.setClusterTolerance (0.02); // 2cm
  
  //Minimum set so that half cut cubes are not classified as clusters
  g_ec.setMinClusterSize (400);
  g_ec.setMaxClusterSize (20000);
  g_ec.setSearchMethod (g_tree_ptr);
  g_ec.setInputCloud (in_cloud_ptr);
  g_ec.extract (g_cluster_indices);
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::extractInlier (PointCPtr &in_cloud_ptr)
{
   /* A function to extract the inliers from the input cloud */


  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

}

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::PointStamped
Cw3Solution::findCubePose (PointCPtr &in_cloud_ptr)
{

  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  g_cube_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cube_pt_msg.header.stamp = ros::Time (0);
  g_cube_pt_msg.point.x = centroid_in[0];
  g_cube_pt_msg.point.y = centroid_in[1];
  g_cube_pt_msg.point.z = centroid_in[2];
  
  // Transform the point to new frame
  geometry_msgs::PointStamped g_cube_pt_msg_out;
  try
  {
    g_listener_.transformPoint ("panda_link0", 
                                g_cube_pt_msg,
                                g_cube_pt_msg_out);
                                
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }
  
  publishPose (g_cube_pt_msg_out);

  g_current_centroid = g_cube_pt_msg_out;


  
  return g_cube_pt_msg_out;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::pubFilteredPCMsg (ros::Publisher &pc_pub,
                               PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
Cw3Solution::publishPose (geometry_msgs::PointStamped &cube_pt_msg)
{
  // Create and publish the cube pose (ignore orientation)

  g_pub_pose.publish (cube_pt_msg);
  
  return;
}
