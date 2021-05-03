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

#ifndef CW3_H_
#define CW3_H_

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// HW1 Includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

// cw3 Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>


/** \brief CW 3.
  *
  * \author Dimitrios Kanoulas
  */
class CW3
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    CW3 (ros::NodeHandle &nh);
    
    /** \brief Lab 1: initialize the parameters. */
    void
    initParams ();
    
    /** \brief Load the parameters. */
    void
    updateParams (ros::NodeHandle &nh);
    
    /** \brief Lab 1: get the world frame.
      *
      * \return the world frame string
      */
    std::string
    getWorldFrame ();
    
    /** \brief Lab 1: get the robot frame.
      *
      * \return the robot frame string
      */
    std::string
    getRobotFrame ();
    
    /** \brief Lab 1: create the transformation between robot & world
     *  frames. 
     */
    void
    Lab1CreateFrames ();
    
    /** \brief Lab 1: publish the transformations between robot & world
      * frames.
      */
    void
    Lab1PublishFrames ();
    
    /** \brief CW1: helper function to implement a non-blocking key getter.
      *
      * \return 1 for key pressed, 0 for no pressed
      */
    int
    kbhit ();
    
    /** \brief CW1 Q2: makes collision object box
      *
      * \input id name of identifier
      * \input frame name of frame_id
      * \input dim_x box dimensions along x
      * \input dim_y box dimensions along y
      * \input dim_z box dimensions along z
      * \input pos_x centre of box along x
      * \input pos_y centre of box along y
      * \input pos_z centre of box along z
      */
    moveit_msgs::CollisionObject
    cw1Q3MakeBox(std::string id, std::string frame_id,
                        float dim_x, float dim_y, float dim_z,
                        float pos_x, float pos_y, float pos_z);

    /** \brief CW1 Q2: add collision objects: table1, table2, table3, object
      *
      * \input planning_scene_interface the MoveIt! PlanningSceneInterface
      */
    void
    cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

    /** \brief check for whether there are objects on the bumper sensors for each table
     * 
     * \input bumper1_pub: publisher for bumper1 sensor information
     *        bumper2_pub: publisher for bumper2 sensor information
     *        bumper3_pub: publisher for bumper3 sensor information
     */
    void
    publishBumpers(ros::Publisher &bumper1_pub, ros::Publisher &bumper2_pub, ros::Publisher &bumper3_pub);

    void
    publishCylinderPose(ros::Publisher &cylinder_pose_pub, geometry_msgs::Pose &cylinder_pose);
    
    void
    updateCylinderPose(ros::Publisher &cylinder_pose_pub, std::string toTable);

    void
    pick (moveit::planning_interface::MoveGroupInterface& move_group, 
                                                      std::string fromTable, bool isTarget);

    void
    place (moveit::planning_interface::MoveGroupInterface& group, 
                                                      std::string toTable, bool isTarget);

  public:
    /** \brief Node handle. */
    ros::NodeHandle nh_;
    
    /** \brief World and robot frames. */
    std::string world_frame_, robot_frame_;
    
    /** \brief Lab 1: TF transforms definition. */
    tf::Transform transf_;
    
    /** \brief Lab 1: TF transform broadcaster definitions. */
    tf::TransformBroadcaster tranf_br_;

    /** \brief bumper sensors boolean variables 
     *  Initialise bumper2HasObject to true and all others to false.
    */
    bool bumper1HasObject = false;
    bool bumper2HasObject = true;
    bool bumper3HasObject = false;

    /** \brief cylinder current positions */
    double objPosX, objPosY, objPosZ;

    /** \brief z values for picking and grasping */
    double Z_GROUND         = 0.2; //0.190;
    double Z_CYLINDER_TABLE = 0.6;
    double Z_OBJECT_TABLE   = 0.68;
    
    /** \brief call back functions*/
    void
    cylinderPoseCB (const geometry_msgs::PoseStamped& msg) 
    {
      ROS_INFO("cylinderPoseCB is called to set the values of objPos");
      // Set the cylinder current position double values for grabbing.
      geometry_msgs::Pose cylinder_pose = msg.pose;
      objPosX = cylinder_pose.position.x;
      objPosY = cylinder_pose.position.y;
      objPosZ = cylinder_pose.position.z;
      return;
    }

    /** \brief subscribers */
    ros::Subscriber cylinder_pos_sub = nh_.subscribe("/cylinder_pose", 1, &CW3::cylinderPoseCB, this);
    
  protected:
    /** \brief Debug mode. */
    bool debug_;
};


#endif
