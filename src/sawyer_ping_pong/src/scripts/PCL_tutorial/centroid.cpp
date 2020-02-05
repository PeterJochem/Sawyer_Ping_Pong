#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>

void processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
void getImage(rs2::pointcloud*, rs2::points* );


/* Describe here 
*/
int main(int argc, char** argv) {	

	rs2::pointcloud pc;
	rs2::points points;

	getImage(&pc, &points);
	//processCloud();
	// getImage();

	return 0;
}


/* Describe here
 * parrallelize this? 
 */ 
void getImage( rs2::pointcloud* pc_ptr, rs2::points* points_ptr) {

	using namespace rs2;

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	// pointcloud pc; // = rs2::context().create_pointcloud();

	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();

	auto data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

	// Wait for the next set of frames from the camera
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();

	// Generate the pointcloud and texture mappings
	*points_ptr = (*pc_ptr).calculate(depth);

	auto color = frames.get_color_frame();

	// Tell pointcloud object to map to this color frame
	(*pc_ptr).map_to(color);

	return;
}


/* Describe here
*/
void processCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width  = 5;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for (std::size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud before filtering: " << std::endl;
	for (std::size_t i = 0; i < cloud->points.size (); ++i)
		std::cerr << "    " << cloud->points[i].x << " " 
			<< cloud->points[i].y << " " 
			<< cloud->points[i].z << std::endl;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
		std::cerr << "    " << cloud_filtered->points[i].x << " " 
			<< cloud_filtered->points[i].y << " " 
			<< cloud_filtered->points[i].z << std::endl;

	// CentroidPoint<pcl::PointXYZ> centroid;
	pcl::CentroidPoint<pcl::PointXYZ> centroid;
	for (int i = 0; i < cloud_filtered->points.size (); ++i) {

		int x = cloud_filtered->points[i].x; 
		int y = cloud_filtered->points[i].y;
		int z = cloud_filtered->points[i].z;

		centroid.add(pcl::PointXYZ (x, y, z) );
	}

	// Fetch centroid using `get()`
	pcl::PointXYZ c1;

	// How to turn a pointCloud into a Centroid point?
	centroid.get (c1);	
	 
		
	// Print c1's data
	std::cerr << "    " << c1.x;


	return;
	// return c1;

	// class pcl::PointCloud<pcl::PointXYZ
	// (*cloud_filtered).get (c1);

	// boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >::get(pcl::PointXYZ&)
	// (cloud_filtered).get (c1);
	// The expected result is: c1.x == 3, c1.y == 4, c1.z == 5
	// It is also okay to use `get()` with a different point type
	// pcl::PointXYZRGB c2;
	// cloud_filtered.get (c2);
	// The expected result is: c2.x == 3, c2.y == 4, c2.z == 5,
	// and c2.rgb is left untouched
}



