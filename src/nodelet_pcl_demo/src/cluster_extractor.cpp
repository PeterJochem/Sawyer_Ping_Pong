#include <iostream>
#include <stdlib.h>
#include <sstream>
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
#include <armadillo>


typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

// Function prototypes 
void computePose(nodelet_pcl_demo::dataPoint);
double calculateTime(double, double);
double computeLocation(double, double, double, double);
geometry_msgs::Pose fitCurve(void);
void plotPlane(double, double, double, double, double, double);

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
		ros::Publisher pose_pub; 
		
		// This is for testing		
		ros::Publisher rate_pub;
		
		// This will store the most recent points observed from the 
		// camera 
		int recentPointsIndex = 0;
		geometry_msgs::PointStamped recentPoints[100];

		// This will publish a custom message type that 
		// will contain the ball's velocity and position
		ros::Publisher dataPoints;

		tf::TransformBroadcaster br;
		tf::TransformListener listener;

		bool hasPublishedVolume;
		
		int planeIDStart = 100000;		

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

		// These define the volume in the robot's frame
		// I store these volume values in the class so that I can avoid 
		// converting to and from frames multiple times
		// This allows me to do one conversion and store the values
		// load these as parameters into the ros server!!!!!

		double filter_minX_robot_frame;
		double filter_maxX_robot_frame;

		double filter_minY_robot_frame;
		double filter_maxY_robot_frame;

		double filter_minZ_robot_frame;
		double filter_maxZ_robot_frame;

		// Same values as above but in the table's frame
		double filter_minX_table_frame;
		double filter_maxX_table_frame;

		double filter_minY_table_frame;
		double filter_maxY_table_frame;

		double filter_minZ_table_frame;
		double filter_maxZ_table_frame;


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

			vis_pub = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
			
			pose_pub = n_.advertise<geometry_msgs::Pose>("/desired_pose", 1);	

			rate_pub = n_.advertise<geometry_msgs::Point>("/proccesed", 100);

			// How big to make the queue? 
			dataPoints = n_.advertise<nodelet_pcl_demo::dataPoint>("/position_and_velocity", 1);	

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

			// Get the filter dimensions from the server
			// These are defined in the launchfile
			// They are defined in terms of the table

			n_.getParam("filter_minX_table_frame", filter_minX_table_frame);
			n_.getParam("filter_maxX_table_frame", filter_maxX_table_frame);

			n_.getParam("filter_minY_table_frame", filter_minY_table_frame);
			n_.getParam("filter_maxY_table_frame", filter_maxY_table_frame);

			n_.getParam("filter_minZ_table_frame", filter_minZ_table_frame);
			n_.getParam("filter_maxZ_table_frame", filter_maxZ_table_frame);

			// Convert the filter dimensions into the base's frame
			geometry_msgs::PointStamped minCorner = convertPointToRobotFrame(filter_minX_table_frame, filter_minY_table_frame, filter_minZ_table_frame);
			geometry_msgs::PointStamped maxCorner = convertPointToRobotFrame(filter_maxX_table_frame, filter_maxY_table_frame, filter_maxZ_table_frame);

			// Unpack the transformed point
			filter_minX_robot_frame = minCorner.point.x;
			filter_minY_robot_frame = minCorner.point.y;
			filter_minZ_robot_frame = minCorner.point.z;

			filter_maxX_robot_frame = maxCorner.point.x;
			filter_maxY_robot_frame = maxCorner.point.y;
			filter_maxZ_robot_frame = maxCorner.point.z;

		}

		/* This method converts a (x, y, z) location from the TABLE'S frame
		 * into the robot's frame. This is for converting the parameters describing
		 * the volume over which we will filter, defined in the camera's frame, into
		 * parameters which define the volume in the camera's frame
		 * Input: doubles x, y, and z which describe a points location in the robot's frame
		 * Returns: geometry_msgs PointStamped of the input point in the camera's frame
		 */
		geometry_msgs::PointStamped convertPointToRobotFrame(double x, double y, double z) {

			bool tableExists = false;

			tf::TransformListener listener;
			tf::StampedTransform transform;

			do {
				try {
					listener.lookupTransform("table", "base", ros::Time(0), transform);
					tableExists = true;
				}
				catch (tf::TransformException ex){
					tableExists = false;
				}
			}

			while ( tableExists == false );


			geometry_msgs::PointStamped point_in_robot_frame;
			geometry_msgs::PointStamped point_in_table_frame;

			point_in_table_frame.header.frame_id = "table";
			point_in_table_frame.header.stamp = ros::Time();

			point_in_table_frame.point.x = x;
			point_in_table_frame.point.y = y;
			point_in_table_frame.point.z = z;

			point_in_robot_frame.header.frame_id = "base";
			point_in_robot_frame.header.stamp = ros::Time();

			listener.transformPoint("base", point_in_table_frame, point_in_robot_frame);

			return point_in_robot_frame;
		}



		/* This method converts a (x, y, z) location from the TABLE'S frame
		 * into the camera's frame. This is for converting the parameters describing
		 * the volume over which we will filter, defined in the camera's frame, into
		 * parameters which define the volume in the camera's frame
		 * Input: doubles x, y, and z which describe a points location in the robot's frame
		 * Returns: geometry_msgs PointStamped of the input point in the camera's frame
		 */ 
		geometry_msgs::PointStamped convertPointToCameraFrame(double x, double y, double z) {

			bool tableExists = false;

			tf::TransformListener listener;
			tf::StampedTransform transform;

			do {
				try {   
					listener.lookupTransform("table", "camera_depth_frame", ros::Time(0), transform);
					tableExists = true;
				}
				catch (tf::TransformException ex){
					tableExists = false;
				}
			}

			while ( tableExists == false );


			geometry_msgs::PointStamped point_in_camera_frame;
			geometry_msgs::PointStamped point_in_table_frame;

			point_in_table_frame.header.frame_id = "table";
			point_in_table_frame.header.stamp = ros::Time();

			point_in_table_frame.point.x = x; 
			point_in_table_frame.point.y = y; 
			point_in_table_frame.point.z = z;

			point_in_camera_frame.header.frame_id = "camera_depth_frame";
			point_in_camera_frame.header.stamp = ros::Time();


			listener.transformPoint("camera_depth_frame", point_in_table_frame, point_in_camera_frame);

			return point_in_camera_frame;
		}


		/* This method publishes a marker at the position defined by (x, y, z)
		 * in the ROBOT BASE frame
		 * Input: double x, double y, double z which describe the desired 
		 * marker's position in the robot base frame
		 * Returns void
		 */
		void publishMarker(double x, double y, double z, bool isPlane) {

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
			
			// These set the color of the marker
                        if ( isPlane == true ) {
                                marker.color.r = 1.0;
                                marker.color.g = 0.0;
                                marker.color.b = 1.0;
                        			
				marker.scale.x = 0.05;
				marker.scale.y = 0.05;
				marker.scale.z = 0.05;
				marker.color.a = 0.5; 

			}
			else { 
				marker.color.r = 0.0;
                                marker.color.g = 0.0;
                                marker.color.b = 1.0;
			
				marker.scale.x = 0.1;
                                marker.scale.y = 0.1;
                                marker.scale.z = 0.1;
                                marker.color.a = 0.5;
			}

			vis_pub.publish( marker );
		}

		/* This mehthod publishes the markers in RVIZ that 
		 * show the volume over which we are filtering
		 * Input: void
		 * Returns: void
		 */
		void publishVolumeMarkers(void) {
				
			// Adding to test the plane plotting
			// plotPlane(0, 0, 1);

			// Define the points that define the volume's min and max dimensions	
			// We will define the volume in the table's frame - but must 
			// convert them to the camera's frame in order to applt the filter
			geometry_msgs::PointStamped min_point_in_table_frame;
			geometry_msgs::PointStamped max_point_in_table_frame;

			// Declare objects to store the point location from above
			// but in the camera's frame
			geometry_msgs::PointStamped min_point_in_camera_frame;
			geometry_msgs::PointStamped max_point_in_camera_frame;

			geometry_msgs::PointStamped min_point_in_robot_frame;
			geometry_msgs::PointStamped max_point_in_robot_frame;

			// Set the points dimensions
			// FIX ME
			// FIX ME - pass the filter dimensions in through the command line
			// filter_minDIMENSION are globals that describe the volume above
			// the table we care about 
			min_point_in_table_frame.point.x = filter_minX_table_frame; 
			min_point_in_table_frame.point.y = filter_minY_table_frame;
			min_point_in_table_frame.point.z = filter_minZ_table_frame;

			max_point_in_table_frame.point.x = filter_maxX_table_frame;
			max_point_in_table_frame.point.y = filter_maxY_table_frame;
			max_point_in_table_frame.point.z = filter_maxZ_table_frame;

			// Convert the points to the camera's frame - needed to apply volume filter
			min_point_in_camera_frame = convertPointToCameraFrame(min_point_in_table_frame.point.x, 
					min_point_in_table_frame.point.y, min_point_in_table_frame.point.z);
			max_point_in_camera_frame = convertPointToCameraFrame(max_point_in_table_frame.point.x, 
					max_point_in_table_frame.point.y, max_point_in_table_frame.point.z); 

			// Convert the table frame's point to the robot base's frame
			min_point_in_robot_frame = convertPointToRobotFrame(min_point_in_table_frame.point.x,
					min_point_in_table_frame.point.y, min_point_in_table_frame.point.z);
			max_point_in_robot_frame = convertPointToRobotFrame(max_point_in_table_frame.point.x,
					max_point_in_table_frame.point.y, max_point_in_table_frame.point.z);


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

			publishMarker(min_point_in_robot_frame.point.x, min_point_in_robot_frame.point.y, min_point_in_robot_frame.point.z, false);
			publishMarker(min_point_in_robot_frame.point.x, min_point_in_robot_frame.point.y, max_point_in_robot_frame.point.z, false);
			publishMarker(min_point_in_robot_frame.point.x, max_point_in_robot_frame.point.y, min_point_in_robot_frame.point.z, false);
			publishMarker(min_point_in_robot_frame.point.x, max_point_in_robot_frame.point.y, max_point_in_robot_frame.point.z, false);

			publishMarker(max_point_in_robot_frame.point.x, min_point_in_robot_frame.point.y, min_point_in_robot_frame.point.z, false);
			publishMarker(max_point_in_robot_frame.point.x, max_point_in_robot_frame.point.y, min_point_in_robot_frame.point.z, false);
			publishMarker(max_point_in_robot_frame.point.x, min_point_in_robot_frame.point.y, max_point_in_robot_frame.point.z, false);
			publishMarker(max_point_in_robot_frame.point.x, max_point_in_robot_frame.point.y, max_point_in_robot_frame.point.z, false);


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
		void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan) {
			
			// This will publish the markers in RVIZ which describe 
			// the volume over which we care about/filter over 
			if ( hasPublishedVolume == false ) {	
				publishVolumeMarkers();
				hasPublishedVolume = true;
				return;
			}

			// Publish to let me know that another set of data has been processed
			geometry_msgs::Point check;
			check.x = recentPointsIndex;
			rate_pub.publish(check);		


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

			// IMPORTaNT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			if ( length > 0) {
				averageX = averageX / length; 
				averageY = averageY / length;
				averageZ = averageZ / length;
			}
			else {
				// If the cloud does not have any points in it, then
				// publish the center to be (0.0, 0.0, 0.0)
				averageX = 0;
				averageY = 0;
				averageZ = 0;
			}

			// This lets the system know we are going to publish a geometry_msg::Point on the topic Ball_Location
			// The int is the number to keep in the buffer before starting to throw away the old messages
			// ros::Publisher position_pub = n_.advertise<geometry_msgs::Point>("/Ball_Location", 1);

			// This object will hold the location and velocity of the ball
			nodelet_pcl_demo::dataPoint currentState;
			currentState.header.stamp = ros::Time::now();

			currentState.position.x = averageX;
			currentState.position.y = averageY;
			currentState.position.z = averageZ;	

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
			
			int numPoints = 10;


			// Check that the computed point is not the 0 point in the CAMERA's frame
			if ( (length > 1) && (recentPointsIndex < numPoints) && (averageX != 0) && (averageY != 0) && (averageZ != 0)  ) {
				// The length > 0 means the point cloud has more than 0 points in it
				recentPoints[recentPointsIndex].point.x = point_in_base_frame.point.x;
				recentPoints[recentPointsIndex].point.y = point_in_base_frame.point.y;
				recentPoints[recentPointsIndex].point.z = point_in_base_frame.point.z;
	

				recentPointsIndex++;
				
				// Publish to let us know a point was observed
		
		
			}
			else if ( (length > 0) && (recentPointsIndex >= numPoints) ) {
				
				bool exceptionOccured = false;		

				// Call function to compute the ball's parabola 
				try {
					fitCurve();		
				}
				catch( ... ) {
					exceptionOccured = true;
				}
		
				// if ( exceptionOccured != true) {
				// Reset the index	
				recentPointsIndex = 0;
			}
			
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

				// Compute the velocities
				currentState.velocity.x = (float(averageX - priorX) ) / (dt);
				currentState.velocity.y = (float(averageY - priorY) ) / (dt);
				currentState.velocity.z = (float(averageZ - priorZ) ) / (dt);

				// velocity_pub.publish(myVelocity);		
				// ros::spinOnce();

				// Update the prior's fields
				priorX = point_in_base_frame.point.x;
				priorY = point_in_base_frame.point.y;
				priorZ = point_in_base_frame.point.z;

				currentState.position.x = point_in_base_frame.point.x;
                                currentState.position.y = point_in_base_frame.point.y;
                                currentState.position.z = point_in_base_frame.point.z;

				// Remember to timestamp the point
				t_prior = ros::Time::now();
				
				if ( currentState.position.z < 2 ) {

					// Publish the velocity and positions as a dataPoint
					dataPoints.publish(currentState);

					pose_pub.publish( computePose(currentState) );
				}

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
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 1.0;
			
			marker.lifetime = ros::Duration(1.0);

			vis_pub.publish( marker );			
	
		}
		
		
		/* Describe this method
		 * Input: 
		 * Return: 
		 */ 
		void plotPlane(double a, double b, double c, double x_On_Plane, double y_On_Plane, double z_On_Plane) {
				
			// The equation of a plane is 	
			// ax + by + c = z;
			
			double increment = 0.01;
			double maxValue = 0.75;
			
			double z = 0;

			int count = 0;
				
			for (double x = x_On_Plane - maxValue; x < maxValue; x = x + increment) {
				for (double y = y_On_Plane - maxValue; y < maxValue; y = y + increment) {
					
					// This depends on how you define the plane
					// IMPORTANT!!
					z = (1 - (a * x) - (b * y) ) / ( c );  	
					
					// publishMarker(x, y, z, true);
					visualization_msgs::Marker marker;
		                        marker.header.frame_id = "base";
                		        marker.header.stamp = ros::Time();
                        		marker.ns = "my_namespace";

                        		marker.id = planeIDStart + count;
                        		planeIDStart = planeIDStart + 1;
					// count = count + 1;

                        		marker.type = visualization_msgs::Marker::SPHERE;
                        		marker.action = visualization_msgs::Marker::ADD;
                        		
					marker.pose.position.x = x;
                        		marker.pose.position.y = y;
                        		marker.pose.position.z = z;
                        		
					marker.pose.orientation.x = 0.0;
                        		marker.pose.orientation.y = 0.0;
                        		marker.pose.orientation.z = 0.0;
                        		marker.pose.orientation.w = 1.0;

                        		// These set the color of the marker
                               		marker.color.r = 0.0;
                               		marker.color.g = 1.0;
                                	marker.color.b = 0.0;

                                	marker.scale.x = 0.05;
                                	marker.scale.y = 0.05;
                                	marker.scale.z = 0.05;
                               		marker.color.a = 0.5;
					
					marker.lifetime = ros::Duration(0.5);

                        		vis_pub.publish( marker );
				}	
			}

		}


		/* Describe this method 
		 * Input 
		 * Returns 
		 */
		geometry_msgs::Pose fitCurve(void) {
			
			// Fit a plane to the data
			using namespace arma;
                        mat A(recentPointsIndex, 3);
			mat X(3, 1); // what we will solve for
			mat B(recentPointsIndex, 1); // These are the Z coordinates of the observed points

			geometry_msgs::PointStamped nextPoint;
			for (int i = 0; i < recentPointsIndex; ++i) {
				
				nextPoint = recentPoints[i];
				A(i, 0) = nextPoint.point.x;
				A(i, 1) = nextPoint.point.y;
				A(i, 2) = nextPoint.point.z;

				B(i, 0) = 1;
			}
			
			A.print();
				
			// Use pinv to get psuedo inverse?	
			X = inv( A.t() * A ) *  A.t() * B;
			
			// Include a point on the plane
			double x = A(3, 0); 
			double y = A(3, 1);
			double z = A(3, 2);

			//if (  rand() > 0.98 ) {
				plotPlane( X(0, 0), X(1, 0), X(2, 0), x, y, z );	
			//}
			// plotPlane(0, 0, 1);
			
			// Figure out where the ball crosses robot's y = 0
			

			// Return the pose
			geometry_msgs::Pose m;
			m.position.x = 1;
			m.position.y = 1;
			m.position.z = 1;

			return m;
		}



		/* Describe this method here
		 * Input: 
		 * Return:
		 */ 
		geometry_msgs::Pose computePose(nodelet_pcl_demo::dataPoint data) {

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
				
			// FIX ME!!!!
			// FIX ME!!!!
			target_pose.position.x = 0.4;
			target_pose.position.y = 0.4;
			target_pose.position.z = 0.4;

			return target_pose;
		}


};


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

/* Describe this method
 * Input: 
 * Return:  
 */
void computePose(nodelet_pcl_demo::dataPoint data) {

	// This is the time until the ball crosses the robot frame's x-axis
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

/* Main function where execution begins
*/
int main(int argc, char **argv) {

	ros::init(argc, argv, "cluster_extractor");

	// FIX ME - add pause to let ROS system get running
	// FIX later	
	// sleep(20);

	ClusterExtractor extractor;

	// Set node to spin indefinitely
	ros::spin();

	return 0;
}

