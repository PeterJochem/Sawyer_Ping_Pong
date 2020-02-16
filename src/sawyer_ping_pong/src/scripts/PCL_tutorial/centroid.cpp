#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>
#include "example.hpp"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Struct for managing rotation of pointcloud view
struct state {
	state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
	ml(false), offset_x(0.0f), offset_y(0.0f) {}
	double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

float3 colors[] { { 0.8f, 0.1f, 0.3f },
	{ 0.1f, 0.9f, 0.5f },
};



// Make the pipe global to avoid reinitializing it
// Initialization of the pipe is very 
rs2::pipeline myPipe;


void processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
void getImage(rs2::pointcloud*, rs2::points*, pcl::PointCloud<pcl::PointXYZ>::Ptr* );
//void pcl_ptr points_to_pcl(rs2::points*);

/* This method converts a realsense pointcloud into a PCL 
 * pointcloud  
 *
 * Arguments: 
 * points is the realsense point cloud 
 *
 * Returns:
 * Returns 
 *
 */
pcl_ptr points_to_pcl(const rs2::points& points) {

	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();

	for (auto& p : cloud->points) {
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	// Write the new cloud to the pointee 
	// *cloud_ptr = cloud;  
	return cloud;
}


/* Main function. At present, there should be no command 
 * line arguments 
 */
int main(int argc, char** argv) {	

	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// This is the pointcloud in RS2 format
	rs2::pointcloud pc;

	// These are points in the RS2 format that are 
	// fields of a pointcloud
	rs2::points points;

	// Start the pipe here
	// This communicates with the camera - avoid doing this
	// operation more than once	
	myPipe.start();



	// This is the point cloud in PCL format 
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_cloud;

	while (app) {
		// getImage(&pc, &points, &PCL_cloud);
		// processCloud(PCL_cloud);
		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;
		// We want the points object to be persistent so we can display the last cloud when a frame drops
		rs2::points points;

		// Declare RealSense pipeline, encapsulating the actual device and sensors
		//rs2::pipeline pipe;
		// Start streaming with default recommended configuration
		//pipe.start();

		// Wait for the next set of frames from the camera
		auto frames = myPipe.wait_for_frames();

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		auto pcl_points = points_to_pcl(points);

		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 0.4);
		pass.filter(*cloud_filtered);

		std::vector<pcl_ptr> layers;
		layers.push_back(pcl_points);
		layers.push_back(cloud_filtered);


		///////////////////////


		//while (app) // Application still alive?
		//{
		draw_pointcloud(app, app_state, layers);

	}



	return 0;
	}

	// Registers the state variable and callbacks to allow mouse control of the pointcloud
	void register_glfw_callbacks(window& app, state& app_state)
	{
		app.on_left_mouse = [&](bool pressed)
		{
			app_state.ml = pressed;
		};

		app.on_mouse_scroll = [&](double xoffset, double yoffset)
		{
			app_state.offset_x += static_cast<float>(xoffset);
			app_state.offset_y += static_cast<float>(yoffset);
		};

		app.on_mouse_move = [&](double x, double y)
		{
			if (app_state.ml)
			{
				app_state.yaw -= (x - app_state.last_x);
				app_state.yaw = std::max(app_state.yaw, -120.0);
				app_state.yaw = std::min(app_state.yaw, +120.0);
				app_state.pitch += (y - app_state.last_y);
				app_state.pitch = std::max(app_state.pitch, -80.0);
				app_state.pitch = std::min(app_state.pitch, +80.0);
			}
			app_state.last_x = x;
			app_state.last_y = y;
		};

		app.on_key_release = [&](int key)
		{
			if (key == 32) // Escape
			{
				app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
			}
		};
	}


	// Handles all the OpenGL calls needed to display the point cloud
	void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
	{
		// OpenGL commands that prep screen for the pointcloud
		glPopMatrix();
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		float width = app.width(), height = app.height();

		glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		gluPerspective(60, width / height, 0.01f, 10.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

		glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
		glRotated(app_state.pitch, 1, 0, 0);
		glRotated(app_state.yaw, 0, 1, 0);
		glTranslatef(0, 0, -0.5f);

		glPointSize(width / 640);
		glEnable(GL_TEXTURE_2D);

		int color = 0;

		for (auto&& pc : points)
		{
			auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

			glBegin(GL_POINTS);
			glColor3f(c.x, c.y, c.z);

			/* this segment actually prints the pointcloud */
			for (int i = 0; i < pc->points.size(); i++)
			{
				auto&& p = pc->points[i];
				if (p.z)
				{
					// upload the point and texture coordinates only for points we have depth data for
					glVertex3f(p.x, p.y, p.z);
				}
			}
			glEnd();
		}

		// OpenGL cleanup
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glPopAttrib();
		glPushMatrix();
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


		// Creating the pipe is SUPER computationally expensive
		// Liekly need to do some communication with the actual camera!!
		//
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		// pipeline pipe;

		// Start streaming with default recommended configuration

		auto data = myPipe.wait_for_frames(); // Wait for next set of frames from the camera

		// Wait for the next set of frames from the camera
		auto frames = myPipe.wait_for_frames();
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
	void processCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud ) {

		pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		// Filter the X	
		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(original_cloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(0.0, 1.0);
		pass.setFilterLimitsNegative(true);
		// pass.filter(*original_cloud);
		// pass.filter(*cloud_filtered);

		// Filter the Y
		pass.setInputCloud(cloud_filtered);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(0.0, 1.0);
		pass.setFilterLimitsNegative(true);
		// pass.filter(*original_cloud);
		//pass.filter(*cloud_filtered);

		// Filter the Z
		pass.setInputCloud(cloud_filtered);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.setFilterLimitsNegative(true);
		// pass.filter(*original_cloud);
		//pass.filter(*cloud_filtered);


		// CentroidPoint<pcl::PointXYZ> centroid;
		pcl::CentroidPoint<pcl::PointXYZ> centroid;
		for (int i = 0; i < cloud_filtered->points.size(); ++i) {

			int x = cloud_filtered->points[i].x; 
			int y = cloud_filtered->points[i].y;
			int z = cloud_filtered->points[i].z;

			centroid.add(pcl::PointXYZ (x, y, z) );
		}

		// Fetch centroid using `get()`
		pcl::PointXYZ c1;

		// How to turn a pointCloud into a Centroid point?
		centroid.get(c1);	

		// Print c1's data
		std::cerr << "The centroid's x, y, z is (" << c1.x << ", " << c1.y << ", " << c1.z <<  ") \n";

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



