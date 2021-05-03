ADD README HERE.

Pre-requisites:
sudo apt-get install ros-melodic-pcl-conversions

This work includes using PCL for cylinder extraction and Moveit! library for automatic pick and place of the cylinder under constraints.

Instructions to use the code:
- press "f" to toggle the Voxel Grid filter (implemented in filterEnvironment).
- press "p" to toggle the fast filter (implemented in fastFilterEnvironment).
- press "1", "2" or "3" to instruct the robot to move the cylinder to the corresponding table 1, 2 or 3 accordingly. 
    * The robot will handle obstruction detection and clearance automatically by moving the object to another table if any.  

Authors:
Mohamed Shire, Hou Nam Chiang, Renjie Zhou