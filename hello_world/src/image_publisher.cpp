#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    VideoCapture capture;
    capture.open("/home/yuxuanzhao/Desktop/v.mp4");
    if(!capture.isOpened()){
        printf("could not load video data... \n");
        return -1;
    }

    int frames = capture.get(CAP_PROP_FRAME_COUNT);
    double fps = capture.get(CAP_PROP_FPS);
    Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
    cout<<frames<<endl;
    cout<<fps<<endl;
    // cout<<size<endl;
    Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(30);
    while(nh.ok()){
        capture>>frame;
        if(!frame.empty()){
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
        ROS_INFO("running!");
        ros::spinOnce();
        loop_rate.sleep();
    }
}