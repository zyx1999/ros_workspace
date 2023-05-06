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


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        ROS_INFO_THROTTLE(1.0, "Callback...");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    Eigen::Matrix<float, -1, -1> signedDistanceMatrix_;
    cv::cv2eigen(cv_ptr->image, signedDistanceMatrix_);
    cv::Mat img_rgb;
    img_rgb = cv_ptr->image;

    cv::imshow("view", img_rgb);
    cv::waitKey(3);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh("~");

    cv::namedWindow("view", CV_WINDOW_NORMAL);
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/my_sdf_demo/signed_distance", 1, imageCallback);
    
    ros::spin();

    cv::destroyWindow("view");
    return 0;
}