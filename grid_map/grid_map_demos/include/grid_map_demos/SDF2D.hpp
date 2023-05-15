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
#include "grid_map_demos/sdfDetect.h"
#include "grid_map_demos/img2PointCloud.h"

class SDF2D{
public:
    SDF2D(ros::NodeHandle& nh);
    void plainMapInit();
    void to2DSignedDistanceMap(grid_map::Matrix& signedDistance);
    void generateSampleGridMap(grid_map::GridMap&, std::string&);
    void publishSignedDistanceMsg(const grid_map::Matrix& signedDistance_);
    void callServer(const grid_map::Matrix& signedDistance_);
    void callback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void mapFromImage();
// private:
    float map_resolution_{0.5};
    int rows_{0}, cols_{0};
    grid_map::Length map_length_{80.0, 60.0};
    grid_map::Position map_position_{0.0, 0.0};
    std::string elevationLayer_;
    std::string pointcloudTopic;
    ros::NodeHandle nh_;
    grid_map::GridMap map;
    ros::Publisher publisher;
    ros::Publisher pointcloudPublisher_;
    ros::Publisher freespacePublisher_;
    ros::Publisher occupiedPublisher_;
    ros::Subscriber sub_extrema_points_;
    sensor_msgs::ImagePtr signedDistanceMsg_;
    image_transport::Publisher imgTransPub;
    image_transport::Subscriber imgTransSub;
};