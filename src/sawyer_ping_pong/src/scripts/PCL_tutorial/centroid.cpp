#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>

void processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
void getImage(rs2::pointcloud*, rs2::points*, pcl::PointCloud<pcl::PointXYZ>::Ptr* );

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


/* Describe this function here
 */
pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}



/* Main function. At present, there should be no command 
 * line arguments 
*/
int main(int argc, char** argv) {	

	// This is the pointcloud in RS2 format
	rs2::pointcloud pc;

	// These are points in the RS2 format that are 
	// fields of a pointcloud
	rs2::points points;

	// This is the point cloud in PCL format 
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_cloud;
	getImage(&pc, &points, &PCL_cloud);
	
	processCloud(PCL_cloud);

	return 0;
}

/* This method takes a pointer to a PCL pointcloud and 
 * filters it so that only points inside the volume defined by
 * parameters x, y, z are in the point cloud
 *
 * Arguments:
 * PCL_cloud is the 
 *
 * Returns:
 *
 */
 
void filterImage(pcl::PointCloud<pcl::PointXYZ>::Ptr* PCL_cloud, double x, double y, double z) {
	
	// Create a new pointcloud ptr?
	// Do not copy onto the stack!


}



/* This method gets a pointcloud from the realsense camera. Calls method
 * to convert the realsense pointcloud format to the PCL pointcloud format.
 * Arguments: 
 * pc_ptr is a pointer to the realsense pointcloud
 * points_ptr is a pointer to realsense points. These are fields of the pointcloud
 * PCL_cloud is a pointer to a PCL format pointcloud
 * Returns:
 * Returns void. Arguments are pointers to support pass by refrence updating
 * Why am I passing by refrence? I want to avoid copying a pointcloud onto the stack
*/ 
void getImage(rs2::pointcloud* pc_ptr, rs2::points* points_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr* PCL_cloud) {

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
		
	// auto pcl_points = points_to_pcl(points);
	// auto pcl_points = points_to_pcl(*points_ptr);
	*PCL_cloud = points_to_pcl(*points_ptr);

	return;
}


/* This method takes the filtered cloud
 *
 *
 *
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



