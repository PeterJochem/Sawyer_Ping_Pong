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
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <thread>

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

// Function prototypes 
void computePose(nodelet_pcl_demo::dataPoint);
double calculateTime(double, double);
double computeLocation(double, double, double, double);
geometry_msgs::Pose fitCurve(void);
void plotPlane(double, double, double, double, double, double);
double predictParabola(double, double, double, double);
double findInitial_Y_Value(geometry_msgs::Point, arma::mat);

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

		ros::Publisher plane_pub;

		ros::Publisher trajectory_pub;

		// Timer field for post processing
		// point cloud data
		ros::Timer timer;

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
		int trajectoryStart = planeIDStart * 5;

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

		// This records if we are ready to fit a plane/parabola
		// to our data
		bool readyToFit = false;

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

			vis_pub = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 300);

			plane_pub = n_.advertise<visualization_msgs::MarkerArray>("/plane", 5); 	

			pose_pub = n_.advertise<geometry_msgs::Pose>("/desired_pose", 1);	

			rate_pub = n_.advertise<geometry_msgs::Point>("/proccesed", 100);

			trajectory_pub = n_.advertise<visualization_msgs::MarkerArray>("/trajectory", 400);

			// How big to make the queue? 
			dataPoints = n_.advertise<nodelet_pcl_demo::dataPoint>("/position_and_velocity", 1);	
			
			
			// Set up a timer to read from the buffer
			timer = n_.createTimer(ros::Duration(1.0), &ClusterExtractor::callback, this);

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
		
		 /* Describe this method
		  */	
		 void callback(const ros::TimerEvent& event) {
                        
			geometry_msgs::Pose desiredPose;

                        if ( readyToFit == true ) {
                                try {
                                        readyToFit = false;
                                        desiredPose = fitCurve();
					// Publish the desired pose
			       		pose_pub.publish(desiredPose);
                                }
                                catch( ... ) {
                                }
                        }

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

			int numPoints = 2;


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

				// DO THIS OUTSIDE THE POINT CLOUD PROCESSING!!
				bool exceptionOccured = false;		

				// Call function to compute the ball's parabola 
				try {
					readyToFit = true;
				}
				catch( ... ) {
					exceptionOccured = true;
				}

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

					// pose_pub.publish( computePose(currentState) );
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

			marker.lifetime = ros::Duration(300.0);

			vis_pub.publish( marker );			


			}


			/* Describe this method here
			*/ 
			double predictParabola(double a, double b, double c, double input) {

				// z = (a)y^2 + (b)y + c
				return (a * pow(input, 2) ) + (b * input) + c; 
			}

			/* Describe this method here
			*/
			double findInitial_Y_Value( geometry_msgs::Point observedPoint_Plane, arma::mat R_robot_p ) {

				arma::mat predicted_point_plane_frame(4, 1);
				predicted_point_plane_frame(0, 0) = observedPoint_Plane.x;
				predicted_point_plane_frame(1, 0) = observedPoint_Plane.y;
				predicted_point_plane_frame(2, 0) = observedPoint_Plane.z;
				predicted_point_plane_frame(3, 0) = 1.0;

				arma::mat predicted_point_robot_frame = R_robot_p * predicted_point_plane_frame;

				// Return the y-value
				return predicted_point_robot_frame(1, 0);
			}


			/* Describe this method here
			*/
			void plotTrajectory(double a, double b, double c, arma::mat observedPoint_Plane, arma::mat R_robot_p) {

				// The parameters a, b, c describe the parabola   
				// z = (a)y^2 + (b)y + c
				
				// Compute the initial x	
				
				double plane_x = observedPoint_Plane(0, 0);	
				double initial_y = observedPoint_Plane(1, 0);

				visualization_msgs::MarkerArray markerarray;
					
				markerIDCount = markerIDCount + 1;

				for (double i = -1.0; i < 1; i = i + 0.05) {

					// double newY = observedPoint_Plane(1, 0); //i + initial_y;
					// double newZ = observedPoint_Plane(2, 0); // predictParabola(a, b, c, newY);
					double newY = i + initial_y;
                                        double newZ = predictParabola(a, b, c, newY);

					// Convert point into the robot's frame 
					arma::mat predicted_point_plane_frame(4, 1);
					predicted_point_plane_frame(0, 0) = plane_x;
					predicted_point_plane_frame(1, 0) = newY;
					predicted_point_plane_frame(2, 0) = newZ;
					predicted_point_plane_frame(3, 0) = 1.0;

					// Convert between the two frames
					arma::mat predicted_point_robot_frame = R_robot_p * predicted_point_plane_frame;


					visualization_msgs::Marker myMarker;
					myMarker.header.frame_id = "base";
					myMarker.header.stamp = ros::Time();
					myMarker.ns = "my_namespace";

					myMarker.id = markerIDCount;
					markerIDCount = markerIDCount + 1;
					

					myMarker.type = visualization_msgs::Marker::SPHERE;
					myMarker.action = visualization_msgs::Marker::ADD;
					
                                        myMarker.pose.position.x = predicted_point_robot_frame(0, 0);
                                        myMarker.pose.position.y = predicted_point_robot_frame(1, 0);
                                        myMarker.pose.position.z = predicted_point_robot_frame(2, 0);


					myMarker.pose.orientation.x = 0.0;
					myMarker.pose.orientation.y = 0.0;
					myMarker.pose.orientation.z = 0.0;
					myMarker.pose.orientation.w = 1.0;

					// These set the color of the marker
					myMarker.color.r = 1.0;
					myMarker.color.g = 0.0;
					myMarker.color.b = 0.0;
					myMarker.scale.x = 0.05;
					myMarker.scale.y = 0.05;
					myMarker.scale.z = 0.05;
					myMarker.color.a = 0.5;
					// myMarker.lifetime = ros::Duration(1.0);
					markerarray.markers.push_back(myMarker);
				}

				trajectory_pub.publish(markerarray);

				return;
			}



			/* Describe this method here
			*/
			geometry_msgs::Pose findIntersection(double a, double b, double c, arma::mat observedPoint_Plane, arma::mat R_robot_p) {

				geometry_msgs::Pose desiredPose;	
				// Set the default to the 0 position
				desiredPose.position.x = 0.0;
				desiredPose.position.y = 0.0;
				desiredPose.position.z = 0.0;

				// The parameters a, b, c describe the parabola   
				// z = (a)y^2 + (b)y + c

				// Compute the initial x

				double plane_x = observedPoint_Plane(0, 0);
				double initial_y = observedPoint_Plane(1, 0); 	

				for (int i = -3; i < 3; i = i + 0.1) {

					double newY = i + initial_y;
					double newZ = predictParabola(a, b, c, newY);	

					// Convert point into the robot's frame 
					arma::mat predicted_point_plane_frame(4, 1);
					predicted_point_plane_frame(0, 0) = plane_x;
					predicted_point_plane_frame(1, 0) = newY;
					predicted_point_plane_frame(2, 0) = newZ;
					predicted_point_plane_frame(3, 0) = 1.0; 

					// Convert between the two frames
					arma::mat predicted_point_robot_frame = R_robot_p * predicted_point_plane_frame;  

					if ( predicted_point_robot_frame(1, 0) > 0 ) {

						// DesiredPose has position and quaterion fields		
						desiredPose.position.x = predicted_point_robot_frame(0, 0); 
						desiredPose.position.y = predicted_point_robot_frame(1, 0);
						desiredPose.position.z = predicted_point_robot_frame(2, 0);
						
						return desiredPose;
					}						
				}
				
					
				return desiredPose;
			}



			/* Describe this method
			 * Input: 
			 * Return: 
			 */ 
			void plotPlane(double a, double b, double c, double x_On_Plane, double y_On_Plane, double z_On_Plane) {

				// The equation of a plane is 	
				// ax + by + c = z;

				double increment = 0.0075;
				double maxValue = 0.30; //0.75;
			
				double z = 0;

				int count = 0;

				visualization_msgs::MarkerArray markerarray;	
				
				for (double x = x_On_Plane - maxValue; x < x_On_Plane + maxValue; x = x + increment) {
					for (double y = y_On_Plane - maxValue; y < y_On_Plane + maxValue; y = y + increment) {

						// This depends on how you define the plane
						// IMPORTANT!!
						z = (1 - (a * x) - (b * y) ) / ( c );  	
						
						// myMarker.action = visualization_msgs::myMarker::ADD;
						// publishMarker(x, y, z, true);
						visualization_msgs::Marker myMarker;
						myMarker.header.frame_id = "base";
						myMarker.header.stamp = ros::Time();
						myMarker.ns = "my_namespace";

						myMarker.id = planeIDStart;
						planeIDStart = planeIDStart + 1;
						count = count + 1;

						myMarker.type = visualization_msgs::Marker::SPHERE;
						myMarker.action = visualization_msgs::Marker::ADD;

						myMarker.pose.position.x = x; //x;
						myMarker.pose.position.y = y; //y;
						myMarker.pose.position.z = z; //z;
							
						myMarker.pose.orientation.x = 0.0;
						myMarker.pose.orientation.y = 0.0;
						myMarker.pose.orientation.z = 0.0;
						myMarker.pose.orientation.w = 1.0;

						// These set the color of the marker
						myMarker.color.r = 0.0;
						myMarker.color.g = 1.0;
						myMarker.color.b = 0.0;
						myMarker.scale.x = 0.025;
						myMarker.scale.y = 0.025;
						myMarker.scale.z = 0.025;
						myMarker.color.a = 0.5;
						myMarker.lifetime = ros::Duration(100.0);
						markerarray.markers.push_back(myMarker);	
					}	
				}
				
				plane_pub.publish(markerarray);

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

				// This will hold the observed points converted to the plane's frame
				mat obs_points_plane[recentPointsIndex];

				geometry_msgs::PointStamped nextPoint;
				for (int i = 0; i < recentPointsIndex; ++i) {

					nextPoint = recentPoints[i];
					A(i, 0) = nextPoint.point.x;
					A(i, 1) = nextPoint.point.y;
					A(i, 2) = nextPoint.point.z;

					B(i, 0) = 1;
				}

				// Use pinv to get psuedo inverse?	
				X = inv( A.t() * A ) *  A.t() * B;

				// Include a point on the plane
				// SHOULD THIS BE FLIPPED?
				double x = A(1, 0); 
				double y = A(1, 1);
				double z = A(1, 2);

				plotPlane( X(0, 0), X(1, 0), X(2, 0), x, y, z );	

				// Construct the rotation matrix 
				mat R_p_robot(4, 4);
				// Set the bottom row
				R_p_robot(3, 0) = 0.0; 
				R_p_robot(3, 1) = 0.0;
				R_p_robot(3, 2) = 0.0;
				R_p_robot(3, 3) = 1.0;
				// Set the first row
				R_p_robot(0, 0) = X(0, 0);
				R_p_robot(0, 1) = - 1 * X(1, 0);
				R_p_robot(0, 2) = 0.0;
				R_p_robot(0, 3) = x;
				// Set the second row
				R_p_robot(1, 0) = X(1, 0);
				R_p_robot(1, 1) = X(0, 0);
				R_p_robot(1, 2) = 0.0;
				R_p_robot(1, 3) = y;
				// Set the third row
				R_p_robot(2, 0) = 0.0;
				R_p_robot(2, 1) = 0.0;
				R_p_robot(2, 2) = 1.0;
				R_p_robot(2, 3) = z;

				// Convert the points to the plane's frame
				for (int i = 0; i < recentPointsIndex; ++i) {

					mat nextPoint(4, 1);
					nextPoint(0, 0) = recentPoints[i].point.x;
					nextPoint(1, 0) = recentPoints[i].point.y;
					nextPoint(2, 0) = recentPoints[i].point.z;
					nextPoint(3, 0) = 1.0;	

					obs_points_plane[i] = R_p_robot * nextPoint; 
				}

				// Fit a parabola to the points
				// Experimentally, the x coordinate of the point in the plane's frame 
				// does not change so our observed points are now a function of 2 variables
				// Construct the arrays to solve for the parabola parameters
				mat observed_y_plane(3, recentPointsIndex);
				mat observed_z_plane(1, recentPointsIndex);
				mat parameters(1, 3);

				// Set the matrices
				// Convert the points to the plane's frame
				for (int i = 0; i < recentPointsIndex; ++i) {

					// z = (a)y^2 + (b)y + c
					observed_y_plane(0, i) = pow( (obs_points_plane[i](1, 0) ), 2);
					observed_y_plane(1, i) = obs_points_plane[i](1, 0); 
					observed_y_plane(2, i) = 1.0;

					observed_z_plane(0, i) = obs_points_plane[i](2, 0);
				}

				// These values define the parabola in the plane
				// z = (a)y^2 + (b)y + c

				parameters = observed_z_plane * ( inv( observed_y_plane.t() * observed_y_plane ) * observed_y_plane.t() );

				// Plot the trajectory with markers
				
				mat R_robot_p = inv(R_p_robot);
				plotTrajectory( parameters(0, 0), parameters(0, 1), parameters(0, 2), obs_points_plane[recentPointsIndex - 1], R_robot_p);

				// Figure out where the ball crosses robot's y = 0
				// Do a line search? Can I transform the normal vector into the plane's frame?

				// Take the function as defined by the parameters, z(y)
				// Start with center point, compute z(y)
				// Convert the point (x, y, z) into the robot's frame
				// Check if the point has crossed the plane of contact - ie y = 0 (in the robot's frame)
				// When we find the point on the parabola that crosses the plane, return it

				// This returns the desired pose
				// findIntersection(double a, double b, double c, arma::mat observedPoint_Plane, arma::mat R_robot_p)
				// Return the pose
				geometry_msgs::Pose m;
				m.position.x = 1;
				m.position.y = 1;
				m.position.z = 1;
				
				// Remember to reset!!
				recentPointsIndex = 0;

				// return m;
				return findIntersection(parameters(0, 0), parameters(0, 1), parameters(0, 2), 
						obs_points_plane[recentPointsIndex - 1], R_robot_p);
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
			
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
			ClusterExtractor extractor;
			
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));	

			ros::spin();

			return 0;
		}

