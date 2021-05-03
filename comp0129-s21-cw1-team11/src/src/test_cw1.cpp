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

/* Student(s): NAME SURNAME */

#include <cw1.h>

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <boost/graph/graph_concepts.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

///////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  /** 
    * Initialize ROS 
    *
    * The ros::init() function needs to see argc and argv so that it can
    * perform any ROS arguments and name remapping that were provided at the
    * command line. For programmatic remappings you can use a different version
    * of init() which takes remappings directly, but for most command-line
    * programs, passing argc and argv is the easiest way to do it. The third
    * argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
  ros::init (argc, argv, "test_cw1");
  
  /**
    * NodeHandle is the main access point to communications with the ROS
    * system. The first NodeHandle constructed will fully initialize this node,
    * and the last NodeHandle destructed will close down the node.
    */
  ros::NodeHandle nh ("~");
  
  // MoveIt! requirement for non-blocking group.move ()
  ros::AsyncSpinner spinner (5);
  spinner.start();
  
  // Create a cw1 object and update the parameters
  CW1 cw1_obj (nh);
  cw1_obj.updateParams (nh);
    
  /** Lab 1 */
  cw1_obj.Lab1CreateFrames ();
  
  /* CW1 Q1-Q3 */
  cw1_obj.cw1Q1GenFrames ();
  
  //Decleare a joint state publisher
  ros::Publisher joint_pub =
    nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 0);
  ros::Publisher ee_pose_pub =
    nh.advertise<geometry_msgs::PoseStamped>("/ee_pose", 1, true);
  
  // MoveIt! operates on sets of joints called "planning groups" and stores
  // them in an object called the `JointModelGroup`. Throughout MoveIt! the
  // terms "planning group" and "joint model group" are used interchangably.
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control
  // and plan for.
  ros::WallDuration(1.0).sleep ();
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group (PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  move_group.setPlanningTime (45.0);
  
  // Create collision objects: two tables, and a cylindric object
  cw1_obj.cw1Q3AddColObj (planning_scene_interface);
  
  /** cw1-Q1: object publisher. */
  ros::Publisher obj_pub =
    nh.advertise<geometry_msgs::PoseStamped> ("/object_pose", 1, true);
  ros::Publisher ee_obj_close_pub =
    nh.advertise<std_msgs::Bool> ("/ee_obj_close", 1, true);
  
  /** cw1-Q3.3: boolean message publisher. */
  ros::Publisher touched_pub =
    nh.advertise<std_msgs::Bool> ("/bumper2_touched", 1, true);

  ros::Rate r (10);
  bool T_23_IsPrint = false;
  while (ros::ok ())
  {
    /* Lab 1 */
    cw1_obj.Lab1PublishFrames ();
    
    // /* HW1-Q2 */  
    if (!T_23_IsPrint){
      cw1_obj.cw1Q1PubFrames ();
      T_23_IsPrint = true;
    }
    
    /* HW1-Q3, cw1 */
    if (cw1_obj.kbhit()) // check if a key is pressed without blocking the while
    {
      int ch = getchar();
      if (ch == 'h') // if h is pressed
      {
        std::cout << "h pressed" << std::endl;
        
        // here, loop is not really needed
        // // loop until the pose is reached
        // for (int i=0; i<8; i++)
        // {
        //   // ToDo: function to be implemented
        //   cw1_obj.cw1Q2HomeRobot (joint_pub);
          
        //   // Hack to reach the pose, normally it
        //   // should be done by comparing joints.
        //   ros::spinOnce ();
        //   ros::Duration(0.5).sleep ();
        // }

        // ToDo: function to be implemented
        cw1_obj.cw1Q2HomeRobot (joint_pub);
        cw1_obj.cw1Q2PrintTFs ();
        
        // Hack to reach the pose, normally it
        // should be done by comparing joints.
        ros::spinOnce ();
        ros::Duration(0.5).sleep ();
      }
      else if (ch == 'p') // if p is pressed
      {
        std::cout << "p pressed" << std::endl;

        
        
        // ToDo: function to be implemented
        cw1_obj.cw1Q2PrintTFs ();
      }
      else if (ch == 'k') // if m is pressed
      {
        std::cout << "k pressed" << std::endl;
        // here, loop is not really  needed
        // // loop until the pose is reached
        // for (int i=0; i<1; i++)
        // {
        //   // ToDo: function to be implemented
        //   cw1_obj.cw1Q2RotJoint (joint_pub);
          
        //   // Hack to reach the pose, normally it
        //   // should be done by comparing joints.
        //   ros::spinOnce ();
        //   ros::Duration(0.5).sleep ();
        // }

        // ToDo: function to be implemented
        cw1_obj.cw1Q2RotJoint (joint_pub);
        
        // Hack to reach the pose, normally it
        // should be done by comparing joints.
        ros::spinOnce ();
        ros::Duration(0.5).sleep ();
      }

      else if (ch == 'e')
      {
        std::cout << "e pressed" << std::endl;
        cw1_obj.cw1Q3EeObjClosePrint (move_group);
      }
      else if (ch == 'r')
      {
        std::cout << "r pressed" << std::endl;
        cw1_obj.cw1Q3ReachObj (move_group);
      }
      else if (ch == 's')
      {
        std::cout << "s pressed" << std::endl;
        cw1_obj.cw1Q3SelectSideGrasp (move_group);
      }
      else if (ch == 't')
      {
        std::cout << "t pressed" << std::endl;
        cw1_obj.cw1Q3SelectTopGrasp (move_group);
      }
      else if (ch == 'g')
      {
        std::cout << "g pressed" << std::endl;
        cw1_obj.cw1Q3GraspObj (move_group);
      }
      else if (ch == 'm')
      {
        std::cout << "m pressed" << std::endl;
        cw1_obj.cw1Q3MoveObj (move_group);
      }
    }
    
    // CW1 Q1-Q3
    cw1_obj.cw1Q2PubHandPose (ee_pose_pub);
    cw1_obj.cw1Q3PubObjPose (obj_pub);
    cw1_obj.cw1Q3PubEeObjClose (ee_obj_close_pub);
    cw1_obj.cw1Q3ObjTouch (move_group, touched_pub);
    
    ros::spinOnce();
    r.sleep();
  }
    
  return (0);
}