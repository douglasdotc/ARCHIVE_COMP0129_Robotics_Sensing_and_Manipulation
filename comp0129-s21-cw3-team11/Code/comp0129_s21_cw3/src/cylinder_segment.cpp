/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Ridhwan Luthra.
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
*   * Neither the name of Ridhwan Luthra nor the names of its
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
*********************************************************************/

/* Author: Ridhwan Luthra */
/* Modified author: Dimitrios Kanoulas */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <cw3.h>
#include <chrono>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      ROS_INFO("Average framerate( %f ): %f Hz.", _WHAT_, double(count)/double(now - last)); \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

//defined in cw3.h
bool isFilterOn     = false;
bool isFastFilterOn = false;

class CylinderSegment
{
public:
  CylinderSegment ()
  {
    // ROS node generation
    ros::NodeHandle nh;
    
    // Initialize subscriber to the raw point cloud
    ros::Subscriber sub                = nh.subscribe ("/camera/depth_registered/points", 1,
                                                       &CylinderSegment::cloudCB, this);
    
    // Initialize subscriber to the boolean value to toggle filterEvironment
    ros::Subscriber filter_env_sub     = nh.subscribe ("/is_filter_env", 1,
                                                       &CylinderSegment::isFilterEnv, this);

    // Initialize subscriber to the boolean value to toggle filterEvironment
    ros::Subscriber fastfilter_env_sub = nh.subscribe ("/is_fastfilter_env", 1,
                                                       &CylinderSegment::isFastFilterEnv, this);

    // Initialize publisher of the cylinder pose message
    cylinder_pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("/cylinder_pose", 1, true);
    
    // Spin
    ros::spin ();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Callback function for filter_env_sub subscriber
   * 
   * @param msg - Pointer to the message from filter_env_sub subscriber.
  **/
  void isFilterEnv (const std_msgs::Bool &msg){
    // Toggle the flag and reset points_not_found to re-render the cylinder
    isFilterOn       = msg.data; 
    points_not_found = true;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Callback function for filter_env_sub subscriber
   * 
   * @param msg - Pointer to the message from filter_env_sub subscriber.
  **/
  void isFastFilterEnv (const std_msgs::Bool &msg){
    // Toggle the flag and reset points_not_found to re-render the cylinder
    isFastFilterOn   = msg.data; 
    points_not_found = true;
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the parameters of the cylinder add the cylinder to the
    * planning scene.
    *
    * @param cylinder_params - Pointer to the struct AddCylinderParams.
    */
  geometry_msgs::Pose addCylinder ()
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // BEGIN_SUB_TUTORIAL add_cylinder
    //  
    // Adding Cylinder to Planning Scene
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "camera_rgb_optical_frame";
    collision_object.id              = "cylinder";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type          = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    /* Setting height of cylinder. */
    primitive.dimensions[0] = cylinder_params->height;
    /* Setting radius of cylinder. */
    primitive.dimensions[1] = cylinder_params->radius;

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(cylinder_params->direction_vec[0],
                                         cylinder_params->direction_vec[1],
                                         cylinder_params->direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;

    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();

    double angle                = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    cylinder_pose.position.x = cylinder_params->center_pt[0];
    cylinder_pose.position.y = cylinder_params->center_pt[1];
    cylinder_pose.position.z = cylinder_params->center_pt[2];

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
    // END_SUB_TUTORIAL
    // Return cylinder_pose for publishing.
    return cylinder_pose;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud containing just the cylinder, compute its
    *        center point and its height and store in cylinder_params.
    *
    *  @param cloud - Pointcloud containing just the cylinder.
    *  @param cylinder_params - Pointer to the struct AddCylinderParams.
    */
  void extractLocationHeight (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    double max_angle_y = 0.0;
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3];
    double highest_point[3];
    // BEGIN_SUB_TUTORIAL extract_location_height
    // Consider a point inside the point cloud and imagine that point is formed
    // on a XY plane where the perpendicular distance from the plane to the
    // camera is Z. |br|
    // The perpendicular drawn from the camera to the plane hits at center of
    // the XY plane. |br|
    // We have the x and y coordinate of the point which is formed on the XY
    // plane. |br|
    // X is the horizontal axis and Y is the vertical axis. |br|
    // C is the center of the plane which is Z meter away from the center of
    // camera and A is any point on the plane. |br|
    // Now we know Z is the perpendicular distance from the point to the
    // camera. |br|
    // If you need to find the  actual distance d from the point to the camera,
    // you should calculate the hypotenuse-
    // |code_start| hypot(point.z, point.x);\ |code_end| |br|
    // angle the point made horizontally- |code_start| atan2(point.z,point.x);\ |code_end| |br|
    // angle the point made Vertically- |code_start| atan2(point.z, point.y);\ |code_end| |br|
    // Loop over the entire pointcloud.
    for (auto const point : cloud->points)
    {
      /* Find the coordinates of the highest point */
      if (atan2(point.z, point.y) < min_angle_y)
      {
        min_angle_y     = atan2(point.z, point.y);
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      /* Find the coordinates of the lowest point */
      else if (atan2(point.z, point.y) > max_angle_y)
      {
        max_angle_y      = atan2(point.z, point.y);
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    /* Store the center point of cylinder */
    cylinder_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
    /* Store the height of cylinder */
    cylinder_params->height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
             pow((lowest_point[2] - highest_point[2]), 2));
    // END_SUB_TUTORIAL
  }

  /////////////////////////////////////////////////////////////////////////////
  /** \brief Filter through the points in the environment to speed up the segmentation.
   *  only get called if filter is turned on.
    *
    * @param cloud - Pointcloud to be filtered through.
    */
  void filterEnvironment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    if (isFilterOn){
      // Use voidgrid filter to downsample the cloud
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud (cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud);
      ROS_INFO("PointCloud after void grid flitering has: %lu data points", cloud->points.size());
    }
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Fast filter through the points in the environment in all 3 axis to speed up the segmentation.
   *  only get called if filter is turned on.
    *
    * @param cloud - Pointcloud to be filtered through.
    */
  void fastFilterEnvironment (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    const char *axis[3] = {"x", "y", "z"};
    float object_pos[3] = {-0.05, 0.16, 0.92};
    float est_error     = 0.25;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    for (int i = 0; i < 3; i++)
    {
      pass.setFilterFieldName (axis[i]);
      pass.setFilterLimits (object_pos[i] - est_error, object_pos[i] + est_error);
      pass.filter (*cloud);
    }
    ROS_INFO("PointCloud after trimming and flitering has: %lu data points", cloud->points.size());
  }
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given a pointcloud extract the ROI(region of interest) defined by the user.
    *
    * @param cloud - Pointcloud whose ROI needs to be extracted.
    */
  void passThroughFilter (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    // min and max values in z axis to keep
    pass.setFilterLimits (0.3, 1.1); //TBD: hard-coded
    pass.filter (*cloud);
    ROS_INFO("PointCloud after simple pass through flitering has: %lu data points", cloud->points.size());
    
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud and pointer cloud_normals compute the point
    * normals and store in cloud_normals.
    *
    * @param cloud - Pointcloud.
    * @param cloud_normals - The point normals once computer will be stored in
    *                        this.
    */
  void computeNormals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr
      tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch (50); //TBD: hard-coded
    ne.compute (*cloud_normals);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the point normals and point indices, extract the normals for
    *        the indices.
    *
    * @param cloud_normals - Point normals.
    * @param inliers_plane - Indices whose normals need to be extracted.
    */
  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                      pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud and indices of the plane, remove the plannar
    * region from the pointcloud.
    *
    * @param cloud - Pointcloud.
    * @param inliers_plane - Indices representing the plane.
    */
  void removePlaneSurface (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr inliers_plane)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients (true);
    segmentor.setModelType (pcl::SACMODEL_PLANE);
    segmentor.setMethodType (pcl::SAC_RANSAC);
    
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations (1000); //TBD: hard-coded
    
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold (0.01); //TBD: hard-coded
    segmentor.setInputCloud (cloud);
    
    /* Create the segmentation object for the planar model and set all the
     * parameters
     */
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    segmentor.segment (*inliers_plane, *coefficients_plane);
    
    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud (cloud);
    extract_indices.setIndices (inliers_plane);
    
    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative (true);
    extract_indices.filter (*cloud);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point
    * normals extract the cylinder from the pointcloud and store the cylinder
    * parameters in coefficients_cylinder.
    *
    * @param cloud - Pointcloud whose plane is removed.
    * @param coefficients_cylinder - Cylinder parameters used to define an
    *                                infinite cylinder will be stored here.
    * @param cloud_normals - Point normals corresponding to the plane on which
    *                        cylinder is kept
    */
  void extractCylinder (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        pcl::ModelCoefficients::Ptr coefficients_cylinder,
                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the
    // parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    segmentor.setOptimizeCoefficients (true);
    segmentor.setModelType (pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType (pcl::SAC_RANSAC);
    
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight (0.1); //TBD: hard-coded
    
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations (10000); //TBD: hard-coded
    
    // tolerance for variation from model
    segmentor.setDistanceThreshold (0.05); //TBD: hard-coded
    
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits (0, 1); //TBD: hard-coded
    segmentor.setInputCloud (cloud);
    segmentor.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment (*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud);
  }
  
  /////////////////////////////////////////////////////////////////////////////
   /** \brief helper function to transform the pose into panda_link0 frame
    *
    * @param cylinder_pose: the pose message 
    */
   geometry_msgs::Pose 
   transformCylinderPose(geometry_msgs::Pose cylinder_pose)
   {
    // Vector with positions in camera frame
    tf::Vector3 poseInCameraFrame(cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
     
    //Creating transform listener & stamped transform to store transformation
    tf::TransformListener tfListener;
    tf::StampedTransform tf2Link0;

    try{
        // buffer the transformation first
        tfListener.waitForTransform("panda_link0", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3.0));
        // obtain the information of transformation
        tfListener.lookupTransform("panda_link0", "camera_rgb_optical_frame", ros::Time(0), tf2Link0);

      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      // Get rotation matrix version of transform
      tf::Matrix3x3 rotM2Link0(tf2Link0.getRotation());
      tf::Vector3 trans2Link0(tf2Link0.getOrigin());
      // ctreate vector object to store the object pose in link0
      tf::Vector3 poseInLink0;

      // Multiply the rotation matrix with the object pose in camera frame
      poseInLink0 = tf::operator*(rotM2Link0, poseInCameraFrame);

      float link0X = poseInLink0.getX() + trans2Link0.getX();
      float link0Y = poseInLink0.getY() + trans2Link0.getY();
      float link0Z = poseInLink0.getZ() + trans2Link0.getZ();

      ROS_INFO("POSITION OF CYLINDER AFTER TRANSFORMATION:%f, %f, %f", link0X, link0Y, link0Z);
      
      // Modify the pose message positions to the transformed values
      cylinder_pose.position.x = link0X;
      cylinder_pose.position.y = link0Y;
      cylinder_pose.position.z = link0Z;

      return cylinder_pose;
   }

  /////////////////////////////////////////////////////////////////////////////
   /** \brief helper function to publish the cylinder pose message
    *
    * @param cylinder_pose: the pose message 
    */
   void publishCylinderPose(geometry_msgs::Pose cylinder_pose)
   {
     geometry_msgs::PoseStamped cylinder_pose_stamped;
     cylinder_pose_stamped.header.frame_id = "panda_link0";
     cylinder_pose_stamped.header.stamp    = ros::Time::now();
     cylinder_pose_stamped.pose            = cylinder_pose;
     cylinder_pose_pub.publish(cylinder_pose_stamped);
   }

  /////////////////////////////////////////////////////////////////////////////
  void cloudCB (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    FPS_CALC ("cloudCB");
    
    // BEGIN_SUB_TUTORIAL callback
    //
    // Perception Related
    // ^^^^^^^^^^^^^^^^^^
    // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for
    // most of the processing.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*input, *cloud);

    // Using passthough filter to get region of interest. A passthrough filter
    // just eliminates the point cloud values which do not lie in the user
    // specified range.

    // check number of points before filtering
    ROS_INFO("PointCloud before flitering has: %lu data points", cloud->points.size());

    // filter using the given dimensions
    passThroughFilter (cloud);

    // downsample the environment
    ROS_INFO("CHECK VALUE FOR isFilterOn: %d", isFilterOn);
    if (isFilterOn)
    {
      filterEnvironment (cloud);
    }
    ROS_INFO("CHECK VALUE FOR isFastFilterOn: %d", isFastFilterOn);
    if (isFastFilterOn)
    {
      fastFilterEnvironment (cloud);
    }
    ROS_INFO("PointCloud after ALL filtering: %lu data points", cloud->points.size());

    // Declare normals and call function to compute point normals.
    pcl::PointCloud<pcl::Normal>::Ptr
      cloud_normals (new pcl::PointCloud<pcl::Normal>);
    computeNormals (cloud, cloud_normals);
    
    // inliers_plane will hold the indices of the point cloud that correspond
    // to a plane.
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    
    // Detect and eliminate the plane on which the cylinder is kept to ease the
    // process of finding the cylinder.
    removePlaneSurface (cloud, inliers_plane);
    
    // We had calculated the point normals in a previous call to computeNormals,
    // now we will be extracting the normals that correspond to the plane on
    // which cylinder lies.
    // It will be used to extract the cylinder.
    extractNormals (cloud_normals, inliers_plane);
    
    // ModelCoefficients will hold the parameters using which we can define a
    // cylinder of infinite length.
    // It has a public attribute |code_start| values\ |code_end| of type
    // |code_start| std::vector< float >\ |code_end|\
    // .
    // |br|
    // |code_start| Values[0-2]\ |code_end| hold a point on the center line of
    // the cylinder. |br|
    // |code_start| Values[3-5]\ |code_end| hold direction vector of the z-axis.
    // |br|
    // |code_start| Values[6]\ |code_end| is the radius of the cylinder.
    pcl::ModelCoefficients::Ptr
      coefficients_cylinder (new pcl::ModelCoefficients);
    
    using std::chrono::high_resolution_clock;
    using std::chrono::duration;
    using std::chrono::microseconds;
    
    auto t1 = high_resolution_clock::now();
    // Extract the cylinder using SACSegmentation.
    extractCylinder (cloud, coefficients_cylinder, cloud_normals);
    auto t2 = high_resolution_clock::now();
    // Calculate total time taken for cylinder extraction in milliseconds
    
    auto time = std::chrono::duration_cast<microseconds>(t2 - t1);

    if (isFilterOn)
    {
      ROS_INFO("Execution time with filterEnv added is %ld microseconds", time.count()); 
    }
    else if (isFastFilterOn) 
    {
      ROS_INFO("Execution time with FastFilterEnv added is %ld microseconds", time.count()); 
    }
    else
    {
      ROS_INFO("Execution time without ANY filter added is %ld microseconds", time.count()); 
    }

    // END_SUB_TUTORIAL
    if (cloud->points.empty())
    {
      ROS_ERROR_STREAM_NAMED ("cylinder_segment", "Can't find the cylindrical component.");
      return;
    }

    if (points_not_found)
    {
      // BEGIN_TUTORIAL
      // CALL_SUB_TUTORIAL callback
      //
      // Storing Relevant Cylinder Values
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // The information that we have in |code_start| coefficients_cylinder\
      // |code_end| is not enough to define our cylinder.
      // It does not have the actual location of the cylinder nor the actual
      // height. |br|
      // We define a struct to hold the parameters that are actually needed for
      // defining a collision object completely.
      // |br|
      // CALL_SUB_TUTORIAL param_struct
      
      cylinder_params                   = new AddCylinderParams();
      // Store the radius of the cylinder.
      cylinder_params->radius           = coefficients_cylinder->values[6];
      // Store direction vector of z-axis of cylinder.
      cylinder_params->direction_vec[0] = coefficients_cylinder->values[3];
      cylinder_params->direction_vec[1] = coefficients_cylinder->values[4];
      cylinder_params->direction_vec[2] = coefficients_cylinder->values[5];
      
      // Extracting Location and Height
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // Compute the center point of the cylinder using standard geometry
      extractLocationHeight(cloud);
      // CALL_SUB_TUTORIAL extract_location_height
      // Use the parameters extracted to add the cylinder to the planning scene
      // as a collision object.
      geometry_msgs::Pose cylinder_pose = addCylinder();

      // Transform the cylinder pose into link0 frame
      cylinder_pose = transformCylinderPose(cylinder_pose);

      // Publish the cylinder pose information after it's found and added to the scene
      publishCylinderPose(cylinder_pose);

      // CALL_SUB_TUTORIAL add_cylinder
      // END_TUTORIAL
      points_not_found = false;
    }
  }

private:
  // BEGIN_SUB_TUTORIAL param_struct
  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };
  
  // Declare a variable of type AddCylinderParams and store relevant values
  // from ModelCoefficients.
  AddCylinderParams* cylinder_params;

  // Declare a publisher for the cylinder pose
  ros::Publisher cylinder_pose_pub;

  // END_SUB_TUTORIAL
  bool points_not_found = true;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");
  
  // Start the segmentor
  CylinderSegment (); 
}