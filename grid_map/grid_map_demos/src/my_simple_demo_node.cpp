#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <string>

using namespace grid_map;

void genSDF(ros::NodeHandle& nodeHandle, grid_map::GridMap& map, std::string& elevationLayer){
    std::string pointcloudTopic("abc");

    //! Pointcloud publisher.
    ros::Publisher pointcloudPublisher_;

    //! Free space publisher.
    ros::Publisher freespacePublisher_;

    //! Occupied space publisher.
    ros::Publisher occupiedPublisher_;

    pointcloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/full_sdf", 1);
    freespacePublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/free_space", 1);
    occupiedPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/occupied_space", 1);
    
    auto& elevationData = map.get(elevationLayer);

    // Generate SDF.
    const float heightMargin{0.1};
    const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
    const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};
    grid_map::SignedDistanceField sdf(map, elevationLayer, minValue, maxValue);

    // Extract as point clouds.
    sensor_msgs::PointCloud2 pointCloud2Msg;
    grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg);
    pointcloudPublisher_.publish(pointCloud2Msg);

    grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float sdfValue) { return sdfValue > 0.0; });
    freespacePublisher_.publish(pointCloud2Msg);

    grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float sdfValue) { return sdfValue <= 0.0; });
    occupiedPublisher_.publish(pointCloud2Msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "my_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // create grid map
    std::string elevationLayer("elevation");
    GridMap map({elevationLayer});
    map.setFrameId("map");
    map.setGeometry(Length(2.1, 2.1), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x % i cells).", 
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));

    ros::Rate rate(30.0);
    while(nh.ok()){
        // add data to grid map
        ros::Time time = ros::Time::now();
        Position center(0, 0);
        double radius = 0.4;
        // iterator through the whole grid_map using GridMapIterator
        for(GridMapIterator it(map); !it.isPastEnd(); ++it){
            // Position position;
            // map.getPosition(*it, position);
            // ROS_INFO_THROTTLE(1.0, "position.x()=%f,  position.y()=%f ", position.x(), position.y());
            map.at(elevationLayer, *it) = 0;
        }
        for (CircleIterator circleIt(map, center, radius); !circleIt.isPastEnd(); ++circleIt) {
            map.at(elevationLayer, *circleIt) = 0.3;
        }
        genSDF(nh, map, elevationLayer);
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