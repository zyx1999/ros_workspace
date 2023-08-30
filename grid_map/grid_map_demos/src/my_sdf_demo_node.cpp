#include <ros/ros.h>
#include "grid_map_demos/SDF2D.hpp"

using namespace grid_map;

int main(int argc, char **argv){
    ros::init(argc, argv, "my_sdf_demo");
    
    ros::NodeHandle nh("~");
    
    SDF2D sdf2d(nh);

    ros::Rate rate(10.0);
    while(sdf2d.nh_.ok()){
        // elevation
        ros::Time time = ros::Time::now();   
        sdf2d.displayMap.setTimestamp(time.toNSec());
        
        // publish map message to visualizer
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(sdf2d.displayMap, message);
        sdf2d.publisher.publish(message);

        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        rate.sleep();
    }
    
    return 0;
}
