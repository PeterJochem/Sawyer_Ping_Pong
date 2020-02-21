#include <iostream>
#include <fstream>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
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

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

/* This class will handle point cloud processing
*/
class ClusterExtractor {

	private:
		ros::NodeHandle n_;
		ros::Subscriber cloud_sub;
		ros::Publisher position_pub;
		ros::Publisher velocity_pub;
		ros::Publisher filtered_cloud_pub;
		ros::Publisher vis_pub;

		tf::TransformBroadcaster br;
		tf::TransformListener listener;

		bool hasPublishedVolume;

		// These are the prior values we read from the system
		// We need these in order to later compute the velocity
		double priorX;
		double priorY;
		double priorZ;

		// This is the prior data point's time stamp data
		// ros::timeNow() returns a type with two fields
		// http://wiki.ros.org/roscpp/Overview/Time
		ros::Time t_prior;

		// Need unique id for each marker in RVIZ
		int markerIDCount;

		// These define the volume in the robot's base frame
		// that we are filtering over
		double filter_minX = 0.0;
		double filter_maxX = 0.75;

		double filter_minY = -1.5;
		double filter_maxY = -0.3;

		double filter_minZ = 0.0;
		double filter_maxZ = 2.0;

		// Values in the camera frame
		// We will set these when we initilaize the object
		double filter_minX_camera_frame = 1.0;
		double filter_maxX_camera_frame = 1.0;

		double filter_minY_camera_frame = 1.0;
		double filter_maxY_camera_frame = 1.0;

		double filter_minZ_camera_frame = 1.0;
		double filter_maxZ_camera_frame = 1.0;

	public:
		// This is the class's constructor
		ClusterExtractor() {

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

			// This records if we have published the volume markers yet
			hasPublishedVolume = false;

			// These are the prior values we read from the system
			// We need these in order to later compute the velocity
			priorX = 0.0;
			priorY = 0.0;
			priorZ = 0.0;

			// This is the prior data point's time stamp data 
			// ros::timeNow() returns a type with two fields
			// http://wiki.ros.org/roscpp/Overview/Time
			ros::Time t_prior;

			// Need unique id for each marker in RVIZ
			markerIDCount = 0;

			filter_minX = 0.0;
			filter_maxX = 0.75;

			filter_minY = -1.5;
			filter_maxY = -0.3;

			filter_minZ = 0.0;
			filter_maxZ = 2.0;

			// Values in the camera frame
			filter_minX_camera_frame = 1.0;
			filter_maxX_camera_frame = 1.0;

			filter_minY_camera_frame = 1.0;
			filter_maxY_camera_frame = 1.0;

			filter_minZ_camera_frame = 1.0;
			filter_maxZ_camera_frame = 1.0;
		}


		/* This method converts a (x, y, z) location from the robot's frame
		 * into the camera's frame
		 * Input: doubles x, y, and z which describe a points location in the robot's frame
		 * Returns: geometry_msgs PointStamped of the input point in the camera's frame
		 */ 
		geometry_msgs::PointStamped convertPointToCameraFrame(double x, double y, double z) {

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

			// Remember to set the time stamp
			point_in_camera_frame.header.stamp = ros::Time();

			listener.transformPoint("camera_depth_frame", point_in_base_frame, point_in_camera_frame);

			return point_in_camera_frame;
		}


		/* This method publishes a marker at the position defined by (x, y, z)
		 * in the ROBOT BASE frame
		 * Input: double x, double y, double z which describe the desired 
		 * marker's position in the robot base frame
		 * Returns void
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
			marker.color.a = 0.5; 

			// These set the color of the marker
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;

			vis_pub.publish( marker );
		}

		/* This mehthod publishes the markers in RVIZ that 
		 * show the volume over which we are filtering
		 * Input: void
		 * Returns: void
		 */
		void publishVolumeMarkers(void) {

			// Define the points that define the volume's min and max dimensions	
			// We will define the volume in the table's frame - but must 
			// convert them to the camera's frame in order to applt the filter
			geometry_msgs::PointStamped min_point_in_robot_frame;
			geometry_msgs::PointStamped max_point_in_robot_frame;

			// Declare objects to store the point location from above
			// but in the camera's frame
			geometry_msgs::PointStamped min_point_in_camera_frame;
			geometry_msgs::PointStamped max_point_in_camera_frame;


			// Set the points dimensions
			// FIX ME
			// FIX ME - pass the filter dimensions in through the command line
			// filter_minDIMENSION are globals that describe the volume above
			// the table we care about 
			min_point_in_robot_frame.point.x = filter_minX; 
			min_point_in_robot_frame.point.y = filter_minY;
			min_point_in_robot_frame.point.z = filter_minZ;

			max_point_in_robot_frame.point.x = filter_maxX;
			max_point_in_robot_frame.point.y = filter_maxY;
			max_point_in_robot_frame.point.z = filter_maxZ;

			// Convert the points to the camera's frame - needed to apply volume filter
			min_point_in_camera_frame = convertPointToCameraFrame(min_point_in_robot_frame.point.x, 
					min_point_in_robot_frame.point.y, min_point_in_robot_frame.point.z);
			max_point_in_camera_frame = convertPointToCameraFrame(max_point_in_robot_frame.point.x, 
					max_point_in_robot_frame.point.y, max_point_in_robot_frame.point.z); 

			// Now update the global variables to avoid recomputing this 
			// transoformation from one frame to another 	
			filter_minX_camera_frame = min_point_in_camera_frame.point.x;
			filter_minY_camera_frame = min_point_in_camera_frame.point.y;
			filter_minZ_camera_frame = min_point_in_camera_frame.point.z;

			filter_maxX_camera_frame = max_point_in_camera_frame.point.x;
			filter_maxY_camera_frame = max_point_in_camera_frame.point.y;
			filter_maxZ_camera_frame = max_point_in_camera_frame.point.z;

			// Convert points for the corners of the volume
			// This is for the visualization in RVIZ	
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

		/* This method simply computes the min of two numbers
		 * Input: Two doubles, a and b
		 * Returns: The minimum of a and b
		 */	
		double min(double a, double b) {

			if (a < b ) {
				return a;
			}

			return b;
		}

		/* This method simply computes the max of two numbers 
		 * Input: Two doubles, a and b. 
		 * Returns the maximum of a and b 
		 */
		double max(double a, double b) {

			if (a > b) {
				return a;
			}

			return b;
		}

		/* This function gets called every time new pcl data comes in
		 * It takes in the point cloud from the camera's frame and applies 
		 * a passthrough filter to it. It then computes the centroid of the 
		 * given point cloud and converts it to the robot's frame. This data 
		 * is then published on the /ball_Location topic 
		 * We then take this data point and compute the velocity in the robot's 
		 * frame. This data is published on the topic /ball_velocity 
		 * Input: scan is the point cloud from the camera's frame 
		 */ 
		void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)    {

			// This will publish the markers in RVIZ which describe 
			// the volume over which we care about/filter over 
			if ( hasPublishedVolume == false ) {	
				publishVolumeMarkers();
				hasPublishedVolume = true;
			}

			using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

			sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			ros::Time tstamp = ros::Time::now();

			// Convert to from realsense ROS format to the pcl format
			pcl::fromROSMsg(*scan, *original_cloud);
			pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

			// Filter the Z 
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(original_cloud);
			pass.setFilterFieldName("z");

			// Need to check which is the min and max since these points are defined in the table
			// frame and the converted into the camera's frame. Theres not guarantee that 
			// the max point in the table's frame ends up being the max point in the camera's frame
			pass.setFilterLimits( min( filter_minZ_camera_frame, filter_maxZ_camera_frame), 
					max(filter_minZ_camera_frame, filter_maxZ_camera_frame ) );

			// Set this to false in order to remove points we 
			// don't want - Otherwise no filtering is shown in RVIZ
			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_filtered);

			// Filter the X
			pass.setInputCloud(cloud_filtered);
			pass.setFilterFieldName("x");
			// See the note about 10 lines above as to why we apply min/max	
			pass.setFilterLimits( min(filter_minX_camera_frame, filter_maxX_camera_frame), 
					max(filter_minX_camera_frame, filter_maxX_camera_frame) );
			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_filtered);

			// Filter the Y
			pass.setInputCloud(cloud_filtered);
			pass.setFilterFieldName("y");
			// See the note about 20 lines up as to why we need to apply min/max
			pass.setFilterLimits( min(filter_minY_camera_frame, filter_maxY_camera_frame), 
					max(filter_minY_camera_frame, filter_maxY_camera_frame) );
			pass.setFilterLimitsNegative(false);
			pass.filter(*cloud_filtered);

			// These numbers describe the average value 
			// across each dimension of the filtered point cloud
			long double averageX = 0.0;  
			long double averageY = 0.0;
			long double averageZ = 0.0;

			// Process cloud to compute the average (i.e. centroid) point in the cloud
			for (int i = 0; i < cloud_filtered->points.size(); ++i) {
				averageX = averageX + cloud_filtered->points[i].x;
				averageY = averageY + cloud_filtered->points[i].y;
				averageZ = averageZ + cloud_filtered->points[i].z;
			}

			float length = float(cloud_filtered->points.size() );

			if ( length > 0) {
				averageX = averageX / length; 
				averageY = averageY / length;
				averageZ = averageZ / length;
			}
			else {
				// If the cloud does not have any points in it, then
				// publish the center to be (0.0, 0.0, 0.0)
				averageX = 0.0;
				averageY = 0.0;
				averageZ = 0.0;
			}

			// This lets the system know we are going to publish a geometry_msg::Point on the topic Ball_Location
			// The int is the number to keep in the buffer before starting to throw away the old messages
			// ros::Publisher position_pub = n_.advertise<geometry_msgs::Point>("/Ball_Location", 1);

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
			listener.transformPoint("base", point_in_camera_frame, point_in_base_frame);

			position_pub.publish(point_in_base_frame);
			// ros::spinOnce();

			// Compute the new velocity
			geometry_msgs::Point myVelocity;
			if ( ( (priorX == 0.0) && (priorY == 0.0) && (priorZ == 0.0) ) ) {
				// Don't publish velocity, simply record the velocity
				// Set the prior states values if the prior states values 
				// are still all zero (on start and if we get a frame that 
				// does not have a valid point cloud in it)
				priorX = point_in_base_frame.point.x;
				priorY = point_in_base_frame.point.y;
				priorZ = point_in_base_frame.point.z;

				// Remember to time stamp the point
				t_prior = ros::Time::now();
			}
			else if( isnan(averageX) == false) {
				// This checks if the point cloud center is valid
				// Compute the velocity and publish it

				// .toSec() converts the time object to a floating point number	
				// This makes computing dt much easier
				float t_now = ros::Time::now().toSec();

				// Compute dt
				float dt = t_now - t_prior.toSec(); 

				myVelocity.x = (float(averageX - priorX) ) / (dt);
				myVelocity.y = (float(averageY - priorY) ) / (dt);
				myVelocity.z = (float(averageZ - priorZ) ) / (dt);

				velocity_pub.publish(myVelocity);		
				// ros::spinOnce();

				// Update the prior's fields
				priorX = point_in_base_frame.point.x;
				priorY = point_in_base_frame.point.y;
				priorZ = point_in_base_frame.point.z;

				// Remember to timestamp the point
				t_prior = ros::Time::now();
			}

			// Publish the marker of the ball's position to RVIZ
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
			marker.color.a = 0.5;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			vis_pub.publish( marker );

			// Compute the position it will cross the plane
			// First, must define the plane to intersect the ball in
		}
};

/* Main function where execution begins
*/
int main(int argc, char **argv) {

	ros::init(argc, argv, "cluster_extractor");

	// FIX ME - add pause to let ROS system get running
	// FIX later	
	sleep(10);

	ClusterExtractor extractor;

	// Set node to spin indefinitely
	ros::spin();

	return 0;
}

