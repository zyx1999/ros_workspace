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

class SDFDescriptor{
public:
    SDFDescriptor(): it_(nh_){
        isub_ = it_.subscribe("/my_sdf_demo/signed_distance", 1, &SDFDescriptor::imageCallback, this);
        ipub_ = it_.advertise("/my_sdf_demo/extrema_points", 1);
    }
    void imageCallback(const sensor_msgs::ImageConstPtr&);
    void toTXT(const std::vector<cv::Mat>&, const std::vector<std::string>&);
    void find_extrema_points(cv::Mat&, cv::Mat&, std::vector<cv::Point>&, cv::Mat&);
    void calculate_gaussian_curvature(const cv::Mat&, int, cv::Mat&);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber isub_;
    image_transport::Publisher ipub_;
    int once_{0};
};