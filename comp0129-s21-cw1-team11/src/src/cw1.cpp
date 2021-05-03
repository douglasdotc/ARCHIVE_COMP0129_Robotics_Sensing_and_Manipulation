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

#include <cw1.h>
#include <Eigen/Geometry>
#include <iomanip>

CW1::CW1 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";
  
  // Topics
  this->robot_frame_ = "/robot_frame";
  
  // HW1-Q2: setup the names for frames {0}, {1}, {2}, and {3}
  // Frame {0}:
  this->frame_0_ = "/frame_0";

  // Frame {1}:
  this->frame_1_ = "/frame_1";

  // Frame {2}:
  this->frame_2_ = "/frame_2";

  // Frame {3}:
  this->frame_3_ = "/frame_3";
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
}


////////////////////////////////////////////////////////////////////////////////
std::string
CW1::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW1::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
int
CW1::kbhit()
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
CW1::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 3.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion
  
  // Note that the rotation can be given in various forms (rotation matrix,
  // axis-angle, etc), but the function setRotation gets a tf::Quaternion,
  // thus when a different type rotation is available, it should be converted
  // to a tf::Quaternion.
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(), 
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q1GenFrames ()
{  
  // create transformations
  // tf::Transform transf_01, transf_02, transf_03;

  tf::Matrix3x3 tf_R_01;
  tf_R_01.setValue(1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0);
  tf::Quaternion tf_q_01;
  tf_R_01.getRotation(tf_q_01);
  transf_01_.setRotation(tf_q_01);
  transf_01_.setOrigin(tf::Vector3(0.0, 1.0, 1.0));

  tf::Matrix3x3 tf_R_02;
  tf_R_02.setValue(1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0);
  tf::Quaternion tf_q_02;
  tf_R_02.getRotation(tf_q_02);
  transf_02_.setRotation(tf_q_02);
  transf_02_.setOrigin(tf::Vector3(-0.5, 1.5, 1.1));

  tf::Matrix3x3 tf_R_03;
  tf_R_03.setValue( 0.0,  1.0,  0.0,
                    1.0,  0.0,  0.0,
                    0.0,  0.0, -1.0);
  tf::Quaternion tf_q_03;
  tf_R_03.getRotation(tf_q_03);
  transf_03_.setRotation(tf_q_03);
  transf_03_.setOrigin(tf::Vector3(-0.5, 1.5, 3.0));


  transf_01_ = tf::StampedTransform(transf_01_, ros::Time::now(), frame_0_, frame_1_);
  transf_02_ = tf::StampedTransform(transf_02_, ros::Time::now(), frame_0_, frame_2_);
  transf_03_ = tf::StampedTransform(transf_03_, ros::Time::now(), frame_0_, frame_3_);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q1PubFrames ()
{
  ros::Rate loop_rate(100);
  for (int i=1; i < 5; i++)
  // while(ros::ok())
  {
    // publish transformations
    cw1Q1GenFrames ();
    transf_01_br_.sendTransform(transf_01_);
    transf_02_br_.sendTransform(transf_02_);
    transf_03_br_.sendTransform(transf_03_);
    loop_rate.sleep();
  }

  try
  {
    listener_.lookupTransform(frame_2_, frame_3_, ros::Time(0), transf_23_);

    tf::Matrix3x3 R_23;
    R_23 = transf_23_.getBasis();
    
    tf::Vector3 d_23;
    d_23 = transf_23_.getOrigin();
    
    Matrix4d T_23;
    T_23(0,3) = d_23[0];
    T_23(1,3) = d_23[1];
    T_23(2,3) = d_23[2];
    for(int i = 0; i <= 2; i++)
    {
      for(int j = 0; j <= 2; j++)
      {
        T_23(i,j) = R_23[i][j];
      }
    }
    // T_23.block<3,3>(0,0) = R_23;
    T_23(3,0) = T_23(3,1) = T_23(3,2) = 0.0;
    T_23(3,3) = 1.0;
    std::cout << "2_T_3:"             << std::endl;
    std::cout << std::setprecision(1) << std::fixed;
    std::cout << T_23                 << std::endl;
  }
  catch (tf2::TransformException &ex) 
  {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    // continue;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2HomeRobot (ros::Publisher &joint_pub)
{
  // setup and publish the joint_state
  // initialise the JointState message to be sent
  sensor_msgs::JointState msg;    
  // define fields
  msg.header.frame_id = "";               // Empty frame ID
  msg.header.stamp = ros::Time::now();    // Assign time
  msg.name.resize(9);                     // A 9 unit size vector
  msg.position.resize(9);                 // A 9 unit size vector

  // assign the names of the joints 
  msg.name = {"panda_finger_joint1", "panda_finger_joint2", "panda_joint1", "panda_joint2",
  "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  // assign values of the home positions
  msg.position = {0.0, 0.0, 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.785398};
  // joint_state pub publish the message
  joint_pub.publish(msg);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2PrintTFs ()
{
  // lookup the transformation and print it
  // create a transform listener to get the message
  tf::TransformListener tfListener;
  // create 2 geometry messages to store the data
  // 1 for transformation from hand(ee) to base(link0)
  // 1 for transformation from hand(ee) to elbow(link2)
  tf::StampedTransform tf2Link0;
  tf::StampedTransform tf2Link2;

  // lookup for the transformation for the two 
  try{
    // buffer the transformation first
    tfListener.waitForTransform("panda_link0", "panda_hand", ros::Time(0), ros::Duration(3.0));
    tfListener.waitForTransform("panda_link2", "panda_hand", ros::Time(0), ros::Duration(3.0));
    // obtain the information of transformation
    tfListener.lookupTransform("panda_link0", "panda_hand", ros::Time(0), tf2Link0);
    tfListener.lookupTransform("panda_link2", "panda_hand", ros::Time(0), tf2Link2);

  }catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // construct the rotation matrix from the quaternion for both
  tf::Matrix3x3 rotM2Base(tf2Link0.getRotation());
  tf::Matrix3x3 rotM2Elbow(tf2Link2.getRotation());

  // create two matrices for the final value
  double T0h[4][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,1}};
  double T2h[4][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,1}};
  // create vectors of transition
  double vec0h[3] = {tf2Link0.getOrigin().x(), tf2Link0.getOrigin().y(), tf2Link0.getOrigin().z()};
  double vec2h[3] = {tf2Link2.getOrigin().x(), tf2Link2.getOrigin().y(), tf2Link2.getOrigin().z()};

  for (int i = 0 ; i< 3; i++){
    //update T0h
    // for all elements with absolute values smaller than 0.001, take it as 0 
    // check for the first element of the row
    if (abs(rotM2Base.getRow(i).x()) < 0.001) {
        T0h[i][0] =0;
    } else{
      T0h[i][0] = rotM2Base.getRow(i).x();
    }
    // check for the second element of the row
    if (abs(rotM2Base.getRow(i).y()) < 0.001) {
        T0h[i][1] =0;
    } else{
      T0h[i][1] = rotM2Base.getRow(i).y();
    }
    // check for the third element of the row
    if (abs(rotM2Base.getRow(i).z()) < 0.001) {
        T0h[i][2] =0;
    } else{
      T0h[i][2] = rotM2Base.getRow(i).z();
    }

    if (abs(vec0h[i]) < 0.001) {
      T0h[i][3] = 0;
    }else{
       T0h[i][3] = vec0h[i];
    }

    //update T2h
    // for all elements with absolute values smaller than 0.001, take it as 0
    // check for the first element of the row
    if (abs(rotM2Elbow.getRow(i).x()) < 0.001) {
        T2h[i][0] =0;
    } else{
      T2h[i][0] = rotM2Elbow.getRow(i).x();
    }
    // check for the second element of the row
    if (abs(rotM2Elbow.getRow(i).y()) < 0.001) {
        T2h[i][1] =0;
    } else{
      T2h[i][1] = rotM2Elbow.getRow(i).y();
    }
    // check for the third element of the row
    if (abs(rotM2Elbow.getRow(i).z()) < 0.001) {
        T2h[i][2] =0;
    } else{
      T2h[i][2] = rotM2Elbow.getRow(i).z();
    }

    if (abs(vec2h[i]) < 0.001) {
      T2h[i][3] = 0;
    }else{
       T2h[i][3] = vec2h[i];
    }
  }



  // print out the matrices
  // first matrix: transformation from panda_hand to the panda_link0, i.e. the base
  // 0_T_h: transformation from panda_hand frame to panda_link0 frame
  std::cout << "0_T_h" << std::endl;
  for (int i = 0; i < 4; i ++){
    std::cout << T0h[i][0] << " " 
            << T0h[i][1] << " " 
            << T0h[i][2] << " " 
            << T0h[i][3] << std::endl;
  }
  
  // print an empty line between the two matrix
  std::cout << "\n" << std::endl;

  // second matrix: transformation from panda_hand to the panda_link2, i.e. the base
  // 2_T_h: transformation from panda_hand frame to panda_link2 frame
  std::cout << "2_T_h" << std::endl;
  for (int i = 0; i < 4; i ++){
    std::cout << T2h[i][0] << " " 
            << T2h[i][1] << " " 
            << T2h[i][2] << " " 
            << T2h[i][3] << std::endl;
  }

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2RotJoint (ros::Publisher &joint_pub)
{
  // setup the joint_state
  // initialise the JointState message to be sent
  sensor_msgs::JointState msg;    
  // define fields
  msg.header.frame_id = "";               // Empty frame ID
  msg.header.stamp = ros::Time::now();    // Assign time
  msg.name.resize(9);                     // A 9 unit size vector
  msg.position.resize(9);                 // A 9 unit size vector

  // assign the names of the joints 
  msg.name = {"panda_finger_joint1", "panda_finger_joint2", "panda_joint1", "panda_joint2",
  "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  // assign values of the home positions
  // add 1.5707 to panda_joint2 on top of the home position
  msg.position = {0.0, 0.0, 0.0, 1.5707, 0.0, -1.5708, 0.0, 1.5708, 0.785398};
  // joint_state pub publish the message
  joint_pub.publish(msg);
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q2PubHandPose (ros::Publisher &ee_pose_pub)
{
  // create tf listener to get the position and orientation from ee to link_0
  tf::TransformListener tfListener;
  // create stamped transform to store the data
  // 1 for transformation from hand(ee) to base(link0)
  tf::StampedTransform tf2Link0;

  // lookup for the transformation for the two 
  try{
    // buffer the transformation first
    tfListener.waitForTransform("panda_link0", "panda_hand", ros::Time(0), ros::Duration(3.0));
    // obtain the information of transformation
    tfListener.lookupTransform("panda_link0", "panda_hand", ros::Time(0), tf2Link0);

  }catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // create and publish the end effector pose message
  geometry_msgs::PoseStamped msg;
  // fill the fields
  msg.header.frame_id = "";               // Empty frame ID
  msg.header.stamp = ros::Time::now();    // Assign time

  // create point and quaternion messages for position and orientation
  // fill in the position data
  geometry_msgs::Point position;
  position.x = tf2Link0.getOrigin().x();
  position.y = tf2Link0.getOrigin().y();
  position.z = tf2Link0.getOrigin().z();
  // fill in the orientation data
  geometry_msgs::Quaternion q;
  q.x = tf2Link0.getRotation().x();
  q.y = tf2Link0.getRotation().y();
  q.z = tf2Link0.getRotation().z();
  q.w = tf2Link0.getRotation().w();

  msg.pose.position = position;
  msg.pose.orientation = q;

  // ee pos pub publish the message
  ee_pose_pub.publish(msg);

  return;

}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& 
                       planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
    collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table: center of the cube. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type =
    collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table: center of the cube. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type =
    collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object: center of the cylinder. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3PubObjPose (ros::Publisher &obj_pub)
{
  //Distinguish whether its pregrasp, grasping, post placing
  //While pregrasp obj coords: 0.5, 0, 0.5
  //While post place obj cords: 0, 0.5, 0.5 (or something like this)
  //During grasping, the obj is to have the same

  //Joint state subscriber to see state of joints
  //ros::Subscriber sub = n.subscribe("/joint_states",1000, jstateCallBack);
  
  obj_pub.publish(objectPose);

  return;
}


////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3PubEeObjClose (ros::Publisher &ee_obj_close_pub)
{
  //initialising the standard bool message thats to be published
  std_msgs::Bool msg;
  msg.data = ee_obj_close_;

  //publishing message
  ee_obj_close_pub.publish(msg);
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3EeObjClosePrint (moveit::planning_interface::MoveGroupInterface& move_group)
{
  if (ee_obj_close_ == false){
    std::cout << "ee_obj_close_: FALSE" << std::endl;

  } else {
    std::cout << "ee_obj_close_: TRUE" << std::endl;

  }
  
  std::cout << "Object-Gripper Dist: " << eucDist << "m" << std::endl;
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3ReachObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose (pose of panda_link8)
  // pose position.x should be 0.095 away from the object position to avoid collision while getting
  // as close as possible
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  // set grasp positions based on static object position on table1
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to go to the pre-grasp position
  move_group.pick("object", grasps);
  return;
}
 
////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3SelectSideGrasp (moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose (pose of panda_link8)
  // pose position.x should be 0.095 away from the object position to avoid collision while getting
  // as close as possible
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

// set grasp positions based on static object position on table1
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to go to the pre-grasp position
  move_group.pick("object", grasps);
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3SelectTopGrasp (moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose (pose of panda_link8)
  // pose position.z should be 0.2 away from the object position to avoid collision while getting
  // as close as possible
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, M_PI, -M_PI / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

// set grasp positions based on static object position on table1
  grasps[0].grasp_pose.pose.position.x = 0.5;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.7;

  // Setting pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to go to the pre-grasp position
  move_group.pick("object", grasps);
  return;
}

///////////////////////////////////////////////////////////////////////////////
void
openGripper (trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}


///////////////////////////////////////////////////////////////////////////////
void
closedGripper (trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3GraspObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  if (ee_obj_close_){
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single
    // grasp. This is essentially useful when using a grasp generator to generate
    // and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose of link8
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;

    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    // set grasp positions based on static object position on table1
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach with respect to link0 */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;
    // Setting post-grasp retreat respect to link0 */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // open the gripper to grap the object
    openGripper(grasps[0].pre_grasp_posture);

    // close the gripper when the object is grapped
    closedGripper(grasps[0].grasp_posture);
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);
    }
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3MoveObj (moveit::planning_interface::MoveGroupInterface& move_group)
{
  // place to put the object on table 2
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id =
    "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id =
    "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  move_group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  move_group.place("object", place_location);
  // END_SUB_TUTORIAL
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW1::cw1Q3ObjTouch (moveit::planning_interface::MoveGroupInterface& move_group, ros::Publisher &touched_pub)
{ 
  // check whether the current object positions is in place
  // object position.z - half the object height (0.1) = table height (0.4)
   //initialising the standard bool message thats to be published
  std_msgs::Bool msg;
  bool touched = false;

  // check for touching with a tolerance of 0.0001
  if (currObjX == 0 && currObjY == 0.5 && currObjZ - 0.4 - 0.1 < 0.0001){
    touched = true;
  }
  msg.data = touched;

  // publish the boolean message
  touched_pub.publish(msg);
  return;
}
