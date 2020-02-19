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

// Globals
geometry_msgs::Point priorLocation;
// Globals


// Function prototypes 
void computePose(geometry_msgs::PointStamped);


int main(int argc, char **argv) {


	ros::init(argc, argv, "myMoveRobot_Node");
	// ClusterExtractor extractor;

	// Subscribe to /ball_Location

	// Pull the time stamps from the data
	// Make sure they exist/make sense
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/ball_Location", 1, computePose);
	
	

	ros::spin();

	return 0;
}


/* Describe this method 
*/
void computePose(geometry_msgs::PointStamped data) {

	// Take the input data and use it to set the moveIt node's code
	
	// Get the velocity
	// Compute the (x,y) location where z = 0
		// Ie where will the ball cross the plane defined by the 
		// board's back edge
	
	return; 

}


