#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "grid_map_demos/sdfDetect.h"

class SDFDescriptor{
public:
    SDFDescriptor(ros::NodeHandle& nh): nh_(nh), it_(nh_){
        // Drop Publisher/Subscriber way
        // isub_ = it_.subscribe("/my_sdf_demo/signed_distance", 1, &SDFDescriptor::imageCallback, this);
        // ipub_ = it_.advertise("/my_sdf_demo/extrema_points", 1);
        // pub_extrema_points_ = nh_.advertise<sensor_msgs::PointCloud>("/my_sdf_demo/extrema_points", 1);
        
        // Now use C/S to pass sdf map
        service_ = nh_.advertiseService("/sdf_service", &SDFDescriptor::srvCallback, this);
    }
    bool srvCallback(grid_map_demos::sdfDetect::Request& req, grid_map_demos::sdfDetect::Response& res);
    void imageCallback(const sensor_msgs::ImageConstPtr&);
    void toTXT(const std::vector<cv::Mat>&, const std::vector<std::string>&);
    void detect_gaussian_curvature_and_eigen(const cv::Mat& src, int ksize, cv::Mat& dst_doh, cv::Mat& dst_eigenvalue1 , cv::Mat& dst_eigenvalue2);
    void find_extrema_points(const cv::Mat& src_sdf, const cv::Mat& src_doh, std::vector<cv::Point>& dst_extrema_points);
    void classify_extrema_points(const std::vector<cv::Point>& src_extrema_points, cv::Mat& src_eigenvalue1 , cv::Mat& src_eigenvalue2, std::vector<std::vector<cv::Point>>& dst);
private:
    int once_{0};
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Publisher pub_extrema_points_;
    image_transport::ImageTransport it_;
    image_transport::Publisher ipub_;
    image_transport::Subscriber isub_;
};