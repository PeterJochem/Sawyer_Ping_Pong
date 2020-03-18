#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nodelet_pcl_demo/PointArray.h>
#include <geometry_msgs/Point.h>


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
typedef pcl::PointXYZ PointT;


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
class PointPublisher
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher point_pub;

public:
    PointPublisher()
        {
            ROS_DEBUG("Creating subscribers and publishers");
            point_pub = n_.advertise<nodelet_pcl_demo::PointArray>("cloud_points", 1);
            cloud_sub = n_.subscribe("cloud_in", 1, &PointPublisher::cloudcb, this);
            return;
        }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan)
        {
            ROS_DEBUG("Cloud receieved");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

            // Convert to pcl
            ROS_DEBUG("Convert incoming cloud to pcl cloud");
            pcl::fromROSMsg(*scan, *cloud);

            // Now we want to iterate through the cloud, extract the XYZ points,
            // and fill out a PointArray message
            nodelet_pcl_demo::PointArray pts;
            pts.header = scan->header;

            for (size_t i=0; i<cloud->points.size(); ++i)
            {
                geometry_msgs::Point point;
                point.x = cloud->points[i].x;
                point.y = cloud->points[i].y;
                point.z = cloud->points[i].z;
                pts.points.push_back(point);
            }
            point_pub.publish(pts);

            return;
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_publisher");
    PointPublisher publisher;

    ros::spin();

    return 0;
}
