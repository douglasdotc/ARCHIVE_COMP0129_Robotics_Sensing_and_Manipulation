/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-, Dimitrios Kanoulas
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

#include <cw3.h>

CW3::CW3 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";
  
  // Topics
  this->robot_frame_ = "/robot_frame";
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
}


////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
int
CW3::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 0.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0));
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(), 
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
moveit_msgs::CollisionObject
CW3::cw1Q3MakeBox(std::string id, std::string frame_id,
					        float dim_x, float dim_y, float dim_z,
					        float pos_x, float pos_y, float pos_z)
{
  // Makes a Box collision object at given location with given dimensions. 

  moveit_msgs::CollisionObject collision_object;
  
  // Add the first table where the cube will originally be kept.
  collision_object.id              = id;
  collision_object.header.frame_id = frame_id;
  
  /* Define the primitive and its dimensions. */
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type          = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dim_x;
  collision_object.primitives[0].dimensions[1] = dim_y;
  collision_object.primitives[0].dimensions[2] = dim_z;

  /* Define the pose of the table: center of the cube. */
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = pos_x;
  collision_object.primitive_poses[0].position.y = pos_y;
  collision_object.primitive_poses[0].position.z = pos_z;
  collision_object.operation                     = collision_object.ADD;

  return collision_object;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::publishBumpers(ros::Publisher &bumper1_pub, 
                    ros::Publisher &bumper2_pub, 
                    ros::Publisher &bumper3_pub)
{
  // Put bumper sensors on the three tables
  // Bumper sensors send boolean messages to indicate whether it's occupied 
  std_msgs::Bool msg1;
  std_msgs::Bool msg2;
  std_msgs::Bool msg3;

  // Fill in message with global bumper sensor variables
  msg1.data = bumper1HasObject;
  msg2.data = bumper2HasObject;
  msg3.data = bumper3HasObject;

  // publish the messages
  bumper1_pub.publish(msg1);
  bumper2_pub.publish(msg2);
  bumper3_pub.publish(msg3);
}

void 
CW3::publishCylinderPose(ros::Publisher &cylinder_pose_pub, geometry_msgs::Pose &cylinder_pose)
{
  geometry_msgs::PoseStamped cylinder_pose_stamped;
  cylinder_pose_stamped.header.frame_id = "panda_link0";
  cylinder_pose_stamped.header.stamp    = ros::Time::now();
  cylinder_pose_stamped.pose            = cylinder_pose;
  cylinder_pose_pub.publish(cylinder_pose_stamped);
}

void
CW3::updateCylinderPose(ros::Publisher &cylinder_pose_pub, std::string toTable)
{
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.position.z = 0.4 + 0.083;

  if (toTable.compare("table1") == 0)
  {
    cylinder_pose.position.x = 0.0;
    cylinder_pose.position.y = 0.5;
  }
  else if (toTable.compare("table2") == 0)
  {
    cylinder_pose.position.x = -0.5;
    cylinder_pose.position.y =  0.0;
  }
  else if (toTable.compare("table3") == 0)
  {
    cylinder_pose.position.x =  0.0;
    cylinder_pose.position.y = -0.5;
  }
  publishCylinderPose(cylinder_pose_pub, cylinder_pose);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw1Q3AddColObj(moveit::planning_interface::PlanningSceneInterface& 
                    planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 4 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);
  
  // Add the first table where the cube will originally be kept.
  collision_objects[0] = CW3::cw1Q3MakeBox("table1", "panda_link0",
                                            0.4, 0.2, 0.4,  // dimension
                                            0.0, 0.5, 0.2); // position
  
  // Add the second table where we will be placing the cube.
  collision_objects[1] = CW3::cw1Q3MakeBox("table2", "panda_link0",
                                             0.2, 0.4, 0.4,  // dimension
					                                  -0.5, 0.0, 0.2); // position
  
  // Add the second table where we will be placing the cube.
  collision_objects[2] = CW3::cw1Q3MakeBox("table3", "panda_link0",
                                            0.4,  0.2, 0.4,  // dimension
                                            0.0, -0.5, 0.2); // position

  // Define the object that we will be manipulating
  collision_objects[3] = CW3::cw1Q3MakeBox("object", "panda_link0",
                                            0.02, 0.02, 0.2, // dimension
                                            -0.5, 0.0, 0.5); // position

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

///////////////////////////////////////////////////////////////////////////////
void
openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0]            = "panda_finger_joint1";
  posture.joint_names[1]            = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0]    = 0.05;
  posture.points[0].positions[1]    = 0.05;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

///////////////////////////////////////////////////////////////////////////////
void
closedGripper (trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0]    = 0.00;
  posture.points[0].positions[1]    = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

///////////////////////////////////////////////////////////////////////////////
void
CW3::pick(moveit::planning_interface::MoveGroupInterface& move_group, 
          std::string fromTable, bool isCylinder)
{
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Set the destination position values based on toTable and which cylinder it is
  double targetPosX, targetPosY, targetPosZ;
  if(isCylinder)
  {
    // If the cylinder is target, use global objPosZ to set the centre
    targetPosZ = Z_CYLINDER_TABLE;
  }
  else
  {
    // Else that the cylinder is obstacle object, use fixed object dimension to set the centre
    targetPosZ = Z_OBJECT_TABLE; 
  }

  // Set X and Y positions for placing
  if (fromTable.compare("table1") == 0)
  {
    // Set location to centre of table 1 with corresponding height
    targetPosX = 0.0;
    targetPosY = 0.5;
  }
  else if (fromTable.compare("table2") == 0)
  {
    // Set location to centre of table 2 with corresponding height
    targetPosX = -0.5;
    targetPosY =  0.0;
  }
  else if (fromTable.compare("table3") == 0)
  {
    // Set location to centre of table 3 with corresponding height
    targetPosX =  0.0;
    targetPosY = -0.5;
  }
  else
  {
    // Set location to the current object location for all three
    targetPosZ = Z_GROUND;
    targetPosX = objPosX;
    targetPosY = objPosY;
  }
  
  grasps[0].grasp_pose.header.frame_id  = "panda_link0";
  grasps[0].grasp_pose.pose.orientation = current_pose.pose.orientation;

  // Set the position based on inputs
  grasps[0].grasp_pose.pose.position.x  = targetPosX;
  grasps[0].grasp_pose.pose.position.y  = targetPosY;
  grasps[0].grasp_pose.pose.position.z  = targetPosZ;

  // Setting pre-grasp approach
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.z        = -1.0;
  grasps[0].pre_grasp_approach.min_distance              = 0.095;
  grasps[0].pre_grasp_approach.desired_distance          = 0.115;

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z        = 1.0;
  grasps[0].post_grasp_retreat.min_distance              = 0.1;
  grasps[0].post_grasp_retreat.desired_distance          = 0.25;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);
  
  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName(fromTable);

  // Call pick to pick up the object using the grasps given
  std::string target = isCylinder ? "cylinder" : "object";
  move_group.pick(target, grasps);
}

///////////////////////////////////////////////////////////////////////////////
void
CW3::place(moveit::planning_interface::MoveGroupInterface& move_group, 
           std::string toTable, bool isCylinder)
{
  geometry_msgs::Pose inter_pose;
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Set the destination position values based on toTable and which cylinder it is
  double targetPosX, targetPosY, targetPosZ;
  if(isCylinder)
  {
    // If the cylinder is target, use global objPosZ to set the centre
    targetPosZ = 0.46;
    place_location[0].place_pose.pose.orientation = current_pose.pose.orientation;
  }
  else
  {
    // Else that the cylinder is obstacle object, use fixed object dimension to set the centre
    targetPosZ = 0.5;
  }

  if (toTable.compare("table1") == 0)
  {
    // Set location to centre of table 1 with corresponding height
    targetPosX = 0.0;
    targetPosY = 0.5;
  }
  else if (toTable.compare("table2") == 0)
  {
    // Set location to centre of table 2 with corresponding height
    targetPosX = -0.5;
    targetPosY =  0.0;
  }
  else
  {
    // Set location to centre of table 3 with corresponding height
    targetPosX =  0.0;
    targetPosY = -0.5;
  }
  
  place_location[0].place_pose.header.frame_id = "panda_link0";
  place_location[0].place_pose.pose.position.x = targetPosX;
  place_location[0].place_pose.pose.position.y = targetPosY;
  place_location[0].place_pose.pose.position.z = targetPosZ;

  // Approach
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z        = -1.0;

  // Retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.y        = 1.0;

  if(isCylinder)
  {
    // Approach
    place_location[0].pre_place_approach.min_distance     = 0.05;
    place_location[0].pre_place_approach.desired_distance = 0.15;

    // Retreat
    place_location[0].post_place_retreat.min_distance     = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;
  }
  else
  {
    // Approach
    place_location[0].pre_place_approach.min_distance     = 0.05;
    place_location[0].pre_place_approach.desired_distance = 0.35;

    // Retreat
    place_location[0].post_place_retreat.min_distance     = 0.05;
    place_location[0].post_place_retreat.desired_distance = 0.55;
  }
  
  // Setting posture of eef after placing object
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  move_group.setSupportSurfaceName(toTable);

  // Call place to place the object using the place locations given.
  std::string target = isCylinder ? "cylinder" : "object";
  move_group.place(target, place_location);
}