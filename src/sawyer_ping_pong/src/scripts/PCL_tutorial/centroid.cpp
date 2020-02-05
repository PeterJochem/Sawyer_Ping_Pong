#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>

int processCloud(void);
void getImage(void);


/* Describe here 
*/
int main(int argc, char** argv) {	

	return 0;
}


/* Describe here
 * parrallelize this? 
 *
 void getImage(void) {

 using namespace rs2;

// Declare pointcloud object, for calculating pointclouds and texture mappings
pointcloud pc = rs2::context().create_pointcloud();

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
points = pc.calculate(depth);

auto color = frames.get_color_frame();

// Tell pointcloud object to map to this color frame
pc.map_to(color);


}

*/

void getImage(void) {
	rs2::pointcloud pc;
	rs2::points points;
	rs2::pipeline pipe;

	// rs2::config cfg;
	//cfg.enable_device_from_file("recording.bag");
	pipe.start();

	int frame_off_interest = 100;
	while (1) {
		if (frame_off_interest <= 0) {
			auto frames = pipe.wait_for_frames();
			auto color = frames.get_color_frame();
			auto depth = frames.get_depth_frame();

			std::cout << "Measurement at [300, 300] = " << std::to_string(depth.get_distance(300, 300)) << std::endl;

			pc.map_to(color);
			points = pc.calculate(depth);
			auto vertices = points.get_vertices();

			std::cout << "Fond " << std::to_string(points.size()) << " Vertices" << std::endl;

		}
	}
}

/* Describe here
*/
int processCloud(void) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

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

	return 0;
}



