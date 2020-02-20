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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nodelet_pcl_demo/dataPoint.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <unistd.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define MAX_CLUSTERS 3
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
std::string filename;

// Need unique id for each marker in RVIZ
int markerIDCount = 0;


// This is the volume to filter over in the ROBOT'S FRAME
// FIX ME - change this so that we define the volume via the launch file 
double filter_minX = 0.0;
double filter_maxX = 0.75;

double filter_minY = -1.5;
double filter_maxY = -0.3;

double filter_minZ = 0.0;
double filter_maxZ = 2.0;

// Values in the camera frame
double filter_minX_camera_frame = 1.0;
double filter_maxX_camera_frame = 1.0;

double filter_minY_camera_frame = 1.0;
double filter_maxY_camera_frame = 1.0;

double filter_minZ_camera_frame = 1.0;
double filter_maxZ_camera_frame = 1.0;


// Describe  these
double maxX = 2.0;
double maxY = 2.0;
double maxZ = 2.0;



// These are the prior values we read from the system
double priorX = 0.0;
double priorY = 0.0;
double priorZ = 0.0;

// I obtained this matrix by measuring the fixed offset

// This is the prior data point's time stamp data 
// ros::timeNow() returns a type with two fields
// http://wiki.ros.org/roscpp/Overview/Time
ros::Time t_prior;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class ClusterExtractor
{

	private:
		ros::NodeHandle n_;
		ros::Subscriber cloud_sub;
		// ros::Publisher cloud_pub[MAX_CLUSTERS];
		//ros::Publisher point_pub[MAX_CLUSTERS];
		ros::Publisher position_pub;
		ros::Publisher velocity_pub;
		ros::Publisher filtered_cloud_pub;
		ros::Publisher vis_pub;

		tf::TransformBroadcaster br;
		tf::TransformListener listener;


	public:
		ClusterExtractor() {
			ROS_DEBUG("Creating subscribers and publishers");
			cloud_sub = n_.subscribe("/camera/depth/color/points", 1, &ClusterExtractor::cloudcb, this);

			br = tf::TransformBroadcaster();

			// Lets the system know we are going to publish a geometry_msg::Point on the topic Ball_Location
			// The int is the number to keep in the buffer before starting to throw away the old messages
			position_pub = n_.advertise<geometry_msgs::PointStamped>("/ball_Location", 3);

			// filtered_cloud_pub = n_.advertise<PCLCloud>("/filtered_cloud", 5);

			// filtered_cloud_pub = n_.advertise<sensor_msgs::PointCloud2> ("/cloud_pcl", 100);
			filtered_cloud_pub = n_.advertise<PCLCloud> ("/cloud_filtered", 1);

			velocity_pub = n_.advertise<geometry_msgs::Point>("/ball_velocity", 3);

			vis_pub = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
			
			// Construct the transform listener?	
			// listener = TransformListener(ros::Duration max_cache_time=ros::Duration(DEFAULT_CACHE_TIME), bool spin_thread=true);
			
			// This publishes the corners of the volume we are filtering over
			// publishVolumeMarkers();
		}
		
		
		/* Describe here
		*/ 
		geometry_msgs::PointStamped convertPointToCameraFrame(double x, double y, double z) {
			

			// listener.transformPoint("base", point_in_camera_frame, point_in_base_frame);
			
			geometry_msgs::PointStamped point_in_camera_frame;
                        geometry_msgs::PointStamped point_in_base_frame;

			point_in_base_frame.header.frame_id = "base";
                        point_in_base_frame.header.stamp = ros::Time();
			point_in_base_frame.point.x = x; 
			point_in_base_frame.point.y = y; 
			point_in_base_frame.point.z = z;

                        point_in_camera_frame.header.frame_id = "camera_depth_frame";
                        point_in_camera_frame.header.stamp = ros::Time();
			
			point_in_base_frame.header.frame_id = "base";
                        point_in_camera_frame.header.stamp = ros::Time();

			listener.transformPoint("camera_depth_frame", point_in_base_frame, point_in_camera_frame);
			
			return point_in_camera_frame;
		}
		
		
		/* Describe here
		 */
		void publishMarker(double x, double y, double z) {

				
			visualization_msgs::Marker marker;
                        marker.header.frame_id = "base";
                        marker.header.stamp = ros::Time();
                        marker.ns = "my_namespace";

                        marker.id = markerIDCount;
                        markerIDCount = markerIDCount + 1;

                        marker.type = visualization_msgs::Marker::SPHERE;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position.x = x;
                        marker.pose.position.y = y;
                        marker.pose.position.z = z;
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.1;
                        marker.scale.y = 0.1;
                        marker.scale.z = 0.1;
                        marker.color.a = 0.5; // Don't forget to set the alpha!
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        vis_pub.publish( marker );
		}

		/* Describe this method here
		 */
		void publishVolumeMarkers(void) {
			
			geometry_msgs::PointStamped min_point_in_robot_frame;
                        geometry_msgs::PointStamped max_point_in_robot_frame;

                        geometry_msgs::PointStamped min_point_in_camera_frame;
                        geometry_msgs::PointStamped max_point_in_camera_frame;
			

			// Set the points
			min_point_in_robot_frame.point.x = filter_minX; 
			min_point_in_robot_frame.point.y = filter_minY;
			min_point_in_robot_frame.point.z = filter_minZ;
			
			max_point_in_robot_frame.point.x = filter_maxX;
                        max_point_in_robot_frame.point.y = filter_maxY;
                        max_point_in_robot_frame.point.z = filter_maxZ;


			min_point_in_camera_frame = convertPointToCameraFrame(min_point_in_robot_frame.point.x, min_point_in_robot_frame.point.y, min_point_in_robot_frame.point.z);
			max_point_in_camera_frame = convertPointToCameraFrame(max_point_in_robot_frame.point.x, max_point_in_robot_frame.point.y, max_point_in_robot_frame.point.z); 
			
			// Now change the global variables to reflect the 
			
			filter_minX_camera_frame = min_point_in_camera_frame.point.x;
			filter_minY_camera_frame = min_point_in_camera_frame.point.y;
			filter_minZ_camera_frame = min_point_in_camera_frame.point.z;

			filter_maxX_camera_frame = max_point_in_camera_frame.point.x;
                        filter_maxY_camera_frame = max_point_in_camera_frame.point.y;
                        filter_maxZ_camera_frame = max_point_in_camera_frame.point.z;
				
			
			// pass.setFilterLimits(-1*(0.86995 + 0.2), 0); // X	
			// pass.setFilterLimits(0, 2*(0.9906 + 0.2) ); // Y 	
			// pass.setFilterLimits(0.0, 2.1082 - 0.1) // Z		
			/*	
			filter_minX_camera_frame = -1*(0.86995 + 0.2);
                        filter_minY_camera_frame = 0.0;
                        filter_minZ_camera_frame = 0.0;

                        filter_maxX_camera_frame = 0.0;
                        filter_maxY_camera_frame = 2*(0.9906 + 0.2) ;
                        filter_maxZ_camera_frame = 2.1082 - 0.1;
			*/

			// Convert points for the corners of the volume
                        // This is for the visualization in RVIZ


			// Convert the volume points 	
			geometry_msgs::PointStamped points[8];
				
			// The filter volume is defined in the robot's frame	
			/*
			points[0] = convertPointToRobotFrame(filter_maxX, filter_maxY, filter_maxZ);
			points[1] = convertPointToRobotFrame(filter_maxX, filter_maxY, filter_minZ);
		        points[2] = convertPointToRobotFrame(filter_maxX, filter_minY, filter_minZ);
			points[3] = convertPointToRobotFrame(filter_minX, filter_maxY, filter_maxZ);
			points[4] = convertPointToRobotFrame(filter_minX, filter_minY, filter_maxZ);
			points[5] = convertPointToRobotFrame(filter_minX, filter_maxY, filter_minZ);
			points[6] = convertPointToRobotFrame(filter_maxX, filter_minY, filter_maxZ);
			points[7] = convertPointToRobotFrame(filter_minX, filter_minY, filter_minZ);
			
					// Fix with array's length
			for (int i = 0; i < 8; ++i) {
				publishMarker( points[i].point.x, points[i].point.y, points[i].point.z  );
			}
			*/
			
			// These define the volume's 8 corners in RVIZ
			// These points are defined in the robot's frame
			publishMarker(filter_maxX, filter_maxY, filter_maxZ);
                        publishMarker(filter_maxX, filter_maxY, filter_minZ);
                        publishMarker(filter_maxX, filter_minY, filter_minZ);
                        publishMarker(filter_minX, filter_maxY, filter_maxZ);
                        publishMarker(filter_minX, filter_minY, filter_maxZ);
                        publishMarker(filter_minX, filter_maxY, filter_minZ);
                        publishMarker(filter_maxX, filter_minY, filter_maxZ);
                        publishMarker(filter_minX, filter_minY, filter_minZ);
			
		}
		
		double min(double a, double b) {
		
			if (a < b ) {
				return a;
			}

			return b;

		}
		double max(double a, double b) {

			if (a > b) {
				return a;
			}
		
			return b;
		}

		// this function gets called every time new pcl data comes in
		void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)    {
			
			publishVolumeMarkers();

			using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

			ROS_DEBUG("Filtered cloud receieved");
			sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZ>);

			// set time stamp and frame id
			ros::Time tstamp = ros::Time::now();

			// Convert to pcl
			ROS_DEBUG("Convert incoming cloud to pcl cloud");
			pcl::fromROSMsg(*scan, *original_cloud);

			////////////////////////////////////////
			// STARTING CLUSTER EXTRACTION    //
			////////////////////////////////////////
			ROS_DEBUG("Begin cluster extraction");

			pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

			// Filter the X 
			// Create the filtering object
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(original_cloud);

			// NOTE THAT THIS IS ALSO Z - FIX THIS!
			pass.setFilterFieldName("z");

			// What are the units? Are these in meters? 
			// pass.setFilterLimits(filter_minZ_camera_frame, filter_maxZ_camera_frame);
			pass.setFilterLimits( min( filter_minZ_camera_frame, filter_maxZ_camera_frame), max(filter_minZ_camera_frame, filter_maxZ_camera_frame ) );
			// pass.setFilterLimits(0.0, 1000);
			// pass.setFilterLimits(0.0, 2.1082 - 0.1);

			// Set this to false in order to remove points we 
			// don't want - Otherwise no filtering is shown in RVIZ
			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_filtered);

			// Filter the X
			pass.setInputCloud(cloud_filtered);
			pass.setFilterFieldName("x");
			
			pass.setFilterLimits( min(filter_minX_camera_frame, filter_maxX_camera_frame), max(filter_minX_camera_frame, filter_maxX_camera_frame) );
			// pass.setFilterLimits(-1*(0.86995 + 0.2), 0);

			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_filtered);

			// Filter the Y
			pass.setInputCloud(cloud_filtered);
			pass.setFilterFieldName("y");

			pass.setFilterLimits( min(filter_minY_camera_frame, filter_maxY_camera_frame), max(filter_minY_camera_frame, filter_maxY_camera_frame) );
			// pass.setFilterLimits(0, 2*(0.9906 + 0.2) );

			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_filtered);

			// Publish the filtered cloud for RVIZ
			filtered_cloud_pub.publish(*cloud_filtered);			
			// FIX ME!!!!!
			// FIX ME!!!!!
			//filtered_cloud_pub.publish(original_cloud);

			ros::spinOnce();


			// CentroidPoint<pcl::PointXYZ> centroid;
			pcl::CentroidPoint<pcl::PointXYZ> centroid;
			long double averageX = 0.0;  
			long double averageY = 0.0;
			long double averageZ = 0.0;
			for (int i = 0; i < cloud_filtered->points.size(); ++i) {

				averageX = averageX + cloud_filtered->points[i].x;
				averageY = averageY + cloud_filtered->points[i].y;
				averageZ = averageZ + cloud_filtered->points[i].z;

				// int x = cloud_filtered->points[i].x;
				// int y = cloud_filtered->points[i].y;
				// int z = cloud_filtered->points[i].z;
				// centroid.add(pcl::PointXYZ (x, y, z) );
			}

			float length = float(cloud_filtered->points.size() );
			if ( length > 0) {
				averageX = averageX / length; 
				averageY = averageY / length;
				averageZ = averageZ / length;
			}
			else {
				averageX = 0.0;
				averageY = 0.0;
				averageZ = 0.0;
			}

			// Fetch centroid using `get()`
			// pcl::PointXYZ c1;
			// How to turn a pointCloud into a Centroid point?
			// centroid.get(c1);

			// ros::NodeHandle n;	
			// Lets the system know we are going to publish a geometry_msg::Point on the topic Ball_Location
			// The int is the number to keep in the buffer before starting to throw away the old messages
			// ros::Publisher position_pub = n_.advertise<geometry_msgs::Point>("/Ball_Location", 1);

			tf::StampedTransform transform;

			nodelet_pcl_demo::dataPoint currentLocation;
			currentLocation.header.stamp = ros::Time::now();
			currentLocation.myPoint.x = averageX;
			currentLocation.myPoint.y = averageY;
			currentLocation.myPoint.z = averageZ;	
			
			geometry_msgs::PointStamped point_in_camera_frame; 
			geometry_msgs::PointStamped point_in_base_frame;

			point_in_camera_frame.header.frame_id = "camera_depth_frame";
			point_in_camera_frame.header.stamp = ros::Time();
			
			point_in_base_frame.header.frame_id = "base";
                        point_in_base_frame.header.stamp = ros::Time();

			point_in_camera_frame.point.x = averageX;
			point_in_camera_frame.point.y = averageY;
			point_in_camera_frame.point.z = averageZ;


			// Convert between the two frames
		 	// void tf::TransformListener::transformPoint(const string&, const PointStamped&, geometry_msgs::PointStamped&) const
			listener.transformPoint("base", point_in_camera_frame, point_in_base_frame);

			// position_pub.publish(currentLocation);
			position_pub.publish(point_in_base_frame);

			// What does this do? 
			// ROS_INFO("%s", msg.data.c_str());

			ros::spinOnce();

			// loop_rate.sleep();

			// Compute the new velocity
			geometry_msgs::Point myVelocity;
			if ( ( (priorX == 0.0) && (priorY == 0.0) && (priorZ == 0.0) ) ) {
				// Don't publish velocity, simply record the velocity
				priorX = point_in_base_frame.point.x;
				priorY = point_in_base_frame.point.y;
				priorZ = point_in_base_frame.point.z;

				t_prior = ros::Time::now();
			}
			else if( isnan(averageX) == false) {
				// Compute the velocity and publish it

				// .toSec() converts the time object to a floating point number	
				float t_now = ros::Time::now().toSec();

				// 1 ns is 10^-9 seconds 
				// float dt = dt_seconds + (  float(dt_nano_seconds ) ;

				float dt = t_now - t_prior.toSec(); 
				
				// FIX ME! - CHANGE TO BASE FRAME!
				myVelocity.x = (float(averageX - priorX) ) / (dt);
				myVelocity.y = (float(averageY - priorY) ) / (dt);
				myVelocity.z = (float(averageZ - priorZ) ) / (dt);
				
				velocity_pub.publish(myVelocity);		
				ros::spinOnce();
			
				// Update the prior's fields
				priorX = point_in_base_frame.point.x;
                                priorY = point_in_base_frame.point.y;
                                priorZ = point_in_base_frame.point.z;

                                t_prior = ros::Time::now();
			}

			visualization_msgs::Marker marker;
			marker.header.frame_id = "base";
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			
			marker.id = markerIDCount;
			markerIDCount = markerIDCount + 1;

			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = point_in_base_frame.point.x;
			marker.pose.position.y = point_in_base_frame.point.y;
			marker.pose.position.z = point_in_base_frame.point.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 0.5; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			vis_pub.publish( marker );
				
			// Compute the position it will cross the plane
			// First, must define the plane to intersect the ball in



		}
};

/* Describe function here
*/
void publishTransform(void) {
	
	// This is done via a node from the launch file
	// double T_Robot_Camera[4][4] = {  {1.0, 0.0, 0.0, 0.86995}, {0.0, 1.0, 0.0, 0.9906}, {0.0, 0.0, -1.0, 2.1082}, {0.0, 0.0, 0.0, 1.0}  };
	double x_offset = 0.86995;
	double y_offset = 0.9906;
	double z_offset = 2.1082;

	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(x_offset, y_offset, z_offset) );
	// This is the origin of the camera frame in Sawyer's frame?
	// So the offset is the offset to the camera frame in Sawyer's frame

	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "camera_depth_frame" ) );
	
	return;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "cluster_extractor");
	// publishTransform();
	// convertVolume();
	//convertVolumeToRobotFrame();
	
	sleep(5);

	ClusterExtractor extractor;

	ros::spin();

	return 0;
}

