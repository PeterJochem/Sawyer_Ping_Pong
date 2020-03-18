#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
//#include <pcl_ros point_cloud.h="">
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Vector3.h>

#include <nodelet_pcl_demo/dataPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>




// Function prototypes 
void computePose(nodelet_pcl_demo::dataPoint);
double calculateTime(double, double);
double computeLocation(double, double, double, double);

// Globals
// moveGroup;

int main(int argc, char **argv) {


	ros::init(argc, argv, "myMoveRobot_Node");
	
	// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html#getting-started
	static const std::string PLANNING_GROUP = "right_arm";


	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/position_and_velocity", 1, computePose);
	ros::spin();

	return 0;
}


/* Describe this method
 * Input: 
 * Return:  
 */
void computePose(nodelet_pcl_demo::dataPoint data) {

	// This is the time until the ball crosses the robot frame's y-axis
	double time = calculateTime( double(data.position.y), double(data.velocity.y) );		

	// Acceleration due to gravity	
	double g = 9.8;

	// Compute 
	double x = computeLocation(time, data.position.x, data.velocity.x, 0.0);

	double z = computeLocation(time, data.position.z, data.velocity.z, g);

	// These are the (x, y, z) locations of the ball in the robot's frame

	// Move the arm to this (x, y, z)	
	geometry_msgs::Pose target_pose;

	// What to set the orientation to?
	target_pose.orientation.w = 1.0;

	target_pose.position.x = x;
	target_pose.position.y = 0;
	target_pose.position.z = z;

	// move_group.setPoseTarget(target_pose1);	

	return; 
}

/* Describe this method here
 * Input: 
 * Returns: 
 */
double calculateTime(double y_position, double y_velocity) {

	// delta_y = (velocity_y * t) + (0.5 * acceleration_y * t^2)	

	return ( (-1 * y_position) / (y_velocity) );
}

/* Describe this method here
 * Inputs: 
 * Returns: 
 */
double computeLocation(double time, double position, double velocity, double acceleration) {

	double delta_position = (velocity * time) + (0.5 * (acceleration * pow(time, 2) ) );

	return delta_position;
}	

