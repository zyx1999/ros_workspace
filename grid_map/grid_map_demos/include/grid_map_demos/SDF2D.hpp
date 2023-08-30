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
#include "grid_map_demos/PointCloud20.h"
#include "grid_map_demos/Point20.h"

class SingleMap;
class SDF2D{
public:
    SDF2D(ros::NodeHandle& nh);
    void mapFromImage(std::shared_ptr<SingleMap>&);
    void displayKeypoints(cv::Mat&, SingleMap&);
    void SDFAlign(std::shared_ptr<SingleMap>&, std::shared_ptr<SingleMap>&);
    void combineTwoMap(std::shared_ptr<SingleMap>&, std::shared_ptr<SingleMap>&, std::shared_ptr<SingleMap>&);
    double minHeight = 0.0;
    double maxHeight = 1.0;
    ros::NodeHandle nh_;
    ros::Publisher publisher;
    ros::ServiceClient client_img2PC;
    ros::ServiceClient client_sdf;
    grid_map_demos::img2PointCloud srv_img2PC;
    grid_map_demos::sdfDetect srv_sdf;
    std::vector<std::shared_ptr<SingleMap>> ptrs;
};

class SingleMap{
public:
    SingleMap():elevationLayer_("elevation"){}
    int rows_{0}, cols_{0};
    float map_resolution_{0.05};
    grid_map::Length map_length_{15.0, 15.0};
    grid_map::Position map_position_{0.0, 0.0};
    std::string elevationLayer_;
    grid_map::GridMap map;
    sensor_msgs::Image img;
    cv::Mat keypoints;
    cv::Mat descriptors;
};