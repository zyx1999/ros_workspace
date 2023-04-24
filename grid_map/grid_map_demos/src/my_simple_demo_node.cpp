#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

int main(int argc, char **argv){
    ros::init(argc, argv, "my_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // create grid map
    GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(Length(1.2, 1.2), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x % i cells).", 
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));

    ros::Rate rate(30.0);
    while(nh.ok()){
        // add data to grid map
        ros::Time time = ros::Time::now();
        Position currentPosition(0, 0);
        double radius = 0.1;
        // iterator through the whole grid_map using GridMapIterator


        for(GridMapIterator it(map); !it.isPastEnd(); ++it){
            Position position;
            map.getPosition(*it, position);
            ROS_INFO_THROTTLE(1.0, "position.x()=%f,  position.y()=%f ", position.x(), position.y());
            map.at("elevation", *it) = 0;
            // ROS_INFO_THROTTLE(1.0, "position.x()=%f,  position.y()=%f ", position.x(), position.y());
            // map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
        }
        
        for (CircleIterator circleIt(map, currentPosition, radius); !circleIt.isPastEnd(); ++circleIt) {
            map.at("elevation", *circleIt) = 0.6;
        }
        Index submapStartIndex(-0.3, -0.3);
        Size submapBufferSize(10, 10);
        for (SubmapIterator smIt(map, submapStartIndex, submapBufferSize); !smIt.isPastEnd(); ++smIt) {
            map.at("elevation", *smIt) = 0.3;
        }
        // Publish grid map
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

        rate.sleep();
    }

    return 0;
}