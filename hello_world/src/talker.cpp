#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <vector>
int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok()){
        double noise;
        if(n.getParam("noise", noise)){
            ROS_INFO("noise is %f", noise);
        } else {
            ROS_WARN("didn't find parameter noise");
        }
        std::string string_var;
        if(n.getParam("string_var", string_var)){
            ROS_INFO("string_var %s", string_var.c_str());
        } else {
            ROS_WARN("No string_var name message");
        }

        std::vector<int> vec;
        if(n.getParam("vec", vec)){
            ROS_INFO("got vec");
        } else {
            ROS_WARN("THere is not vec");
        }
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}