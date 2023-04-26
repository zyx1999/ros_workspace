#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>

using namespace grid_map;

int main(int argc, char **argv){
    ros::init(argc, argv, "my_sdf_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    
    GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(Length(2.0, 2.0), 0.05, Position(0.0, 0.0));
    ROS_INFO("Create map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.", 
      map.getLength().x(), map.getLength().y(),
      map.getSize()(0), map.getSize()(1),
      map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());
    
    ros::Rate rate(30.0);
    while(nh.ok()){
        ros::Time time = ros::Time::now();
        int i = 0;
        for(GridMapIterator it(map); !it.isPastEnd(); ++it){
            Position pos;
            map.getPosition(*it, pos);
            map.at("elevation", *it) = (i%3)/3.0;
            ++i;
        }
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        rate.sleep();
    }    
    
    return 0;
}
