#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv) {

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



