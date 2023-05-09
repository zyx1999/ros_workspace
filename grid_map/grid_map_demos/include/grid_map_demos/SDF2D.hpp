#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include "grid_map_sdf/SignedDistance2d.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

class SDF2D{
public:
    SDF2D();
    void generateSampleGridMap(grid_map::GridMap&, std::string&);
    void callback(const sensor_msgs::PointCloud::ConstPtr& msg);
// private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    std::string elevationLayer_;
    grid_map::GridMap map;
    float resolution_;
    std::string pointcloudTopic;
    ros::Publisher pointcloudPublisher_;
    ros::Publisher freespacePublisher_;
    ros::Publisher occupiedPublisher_;
    sensor_msgs::ImagePtr signedDistanceMsg_;
    image_transport::Publisher imgTransPub;
    image_transport::Subscriber imgTransSub;
    ros::Subscriber sub_extrema_points_;
    int rows_{0}, cols_{0};
};