#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
    }
    cv::Mat img_rgb;
    img_rgb = cv_ptr->image;

    cv::imshow("view", img_rgb);
    cv::waitKey(3);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "image_subscriber");
    ros::NodeHandle nh;
    cv::namedWindow("view", CV_WINDOW_NORMAL);
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
    return 0;
}