#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include "grid_map_sdf/SignedDistance2d.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace grid_map;

class SDF2D{
public:
    SDF2D(): nh("~"), elevationLayer_("elevation"), resolution_(0.5), pointcloudTopic("pointcloud_topic"){
        publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
        // map init.
        map.add(elevationLayer_);
        map.setFrameId("map");
        map.setGeometry(Length(30.0, 20.0), resolution_, Position(0.0, 0.0));
        ROS_INFO("Create map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.", 
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1),
        map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

        // sdf init.
        pointcloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/full_sdf", 1);
        freespacePublisher_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/free_space", 1);
        occupiedPublisher_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloudTopic + "/occupied_space", 1);

        generateSampleGridMap(map, elevationLayer_);

        // // Add noise layer (using Eigen operators).
        // map.add("noise", 0.1 * Matrix::Random(map.getSize()(0), map.getSize()(1)));
        // // Add elevation_noisy layer
        // map.add("elevation_noisy", map.get("elevation") + map["noise"]);
        // auto& elevationData = map.get("elevation_noisy");
        
        // elevationData is Matrix
        auto& elevationData = map.get(elevationLayer_);

        // Inpaint if needed.
        if (elevationData.hasNaN()) {
            const float inpaint{elevationData.minCoeffOfFinites()};
            ROS_WARN("[SdfDemo] Map contains NaN values. Will apply inpainting with min value.");
            elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v)? v : inpaint; });
        }

        // Generate 2D SDF.
        Eigen::Matrix<bool, -1, -1> occupancy = elevationData.unaryExpr([=](float val) { return val > 0.5; });
        auto signedDistance = grid_map::signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution_);
        map.add("sdf2d", signedDistance);

        // publish to image_converter with topic: /my_sdf_demo/signed_distance/image_topics
        cv::Mat signedDistanceMat_;
        cv::eigen2cv(signedDistance, signedDistanceMat_);
        signedDistanceMsg_ = cv_bridge::CvImage(std_msgs::Header(), 
            sensor_msgs::image_encodings::TYPE_32FC1, signedDistanceMat_).toImageMsg();
        image_transport::ImageTransport imgTrans(nh);
        imgTransPub = imgTrans.advertise("signed_distance", 1);
        imgTransSub = imgTrans.subscribe("extrema_points", 1, boost::bind(&SDF2D::callback, this, _1));
    }
    void generateSampleGridMap(grid_map::GridMap&, std::string&);
    void callback(const sensor_msgs::ImageConstPtr&);
// private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    std::string elevationLayer_;
    GridMap map;
    float resolution_;
    std::string pointcloudTopic;
    ros::Publisher pointcloudPublisher_;
    ros::Publisher freespacePublisher_;
    ros::Publisher occupiedPublisher_;
    sensor_msgs::ImagePtr signedDistanceMsg_;
    image_transport::Publisher imgTransPub;
    image_transport::Subscriber imgTransSub;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "my_sdf_demo");

    SDF2D sdf2d;

    ros::Rate rate(30.0);
    while(sdf2d.nh.ok()){
        // elevation
        ros::Time time = ros::Time::now();   
        sdf2d.map.setTimestamp(time.toNSec());

        // publish sdf to Converter, then wait until subscribe the return message
        sdf2d.imgTransPub.publish(sdf2d.signedDistanceMsg_);
        ros::spinOnce();
        
        // publish map message to visualizer
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(sdf2d.map, message);
        sdf2d.publisher.publish(message);

        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        rate.sleep();
    }
    
    return 0;
}
void SDF2D::callback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        ROS_INFO_THROTTLE(1.0, "callback1...");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    cv::Mat extrema_src_ = cv_ptr->image;
    Eigen::Matrix<float, -1, -1> extrema_data_;
    // convert Mat into Matrix
    cv::cv2eigen(extrema_src_, extrema_data_);
    map.add("extrema", extrema_data_);
}
void SDF2D::generateSampleGridMap(grid_map::GridMap& map, std::string& elevationLayer_){
    // generate grid_map
    for(GridMapIterator it(map); !it.isPastEnd(); ++it){
        map.at(elevationLayer_, *it) = 0;
    }
    // // submap
    // Index submapStartIndex(3, 5);
    // Index submapBufferSize(12, 7);
    // for (grid_map::SubmapIterator it(map, submapStartIndex, submapBufferSize);
    //     !it.isPastEnd(); ++it) {
    //     map.at(elevationLayer_, *it) = 1.0;
    // }
    // // circle
    // Position center1(0.0, 0.0);
    // double radius1 = 0.4;
    // for (grid_map::CircleIterator it(map, center1, radius1);
    //     !it.isPastEnd(); ++it) {
    //     map.at(elevationLayer_, *it) = 1.0;
    // }
    // // Ellipse      
    // Position center2(-1.0, -1.0);
    // Length length(0.45, 0.9);
    // for (grid_map::EllipseIterator it(map, center2, length, M_PI_4);
    //     !it.isPastEnd(); ++it) {
    //     map.at(elevationLayer_, *it) = 1.0;
    // }
    // line
    // Index start1(-18, 2);
    // Index end1(-2, 13);
    // for (grid_map::LineIterator it(map, start1, end1);
    //     !it.isPastEnd(); ++it) {
    //     map.at(elevationLayer_, *it) = 1.0;
    // }
    // Index start2(-18, 1);
    // Index end2(-2, 12);
    // for (grid_map::LineIterator it(map, start2, end2);
    //     !it.isPastEnd(); ++it) {
    //     map.at(elevationLayer_, *it) = 1.0;
    // }
    // box
    Index lt(5, 5), rt(5, 25), lb(46, 5), rb(46, 25);
    for(grid_map::LineIterator it(map, lt, rt); !it.isPastEnd(); ++it){
        map.at(elevationLayer_, *it) = 1.0;
    }
    for(grid_map::LineIterator it(map, lb, rb); !it.isPastEnd(); ++it){
        map.at(elevationLayer_, *it) = 1.0;
    } 
    for(grid_map::LineIterator it(map, lt, lb); !it.isPastEnd(); ++it){
        map.at(elevationLayer_, *it) = 1.0;
    } 
    for(grid_map::LineIterator it(map, rt, rb); !it.isPastEnd(); ++it){
        map.at(elevationLayer_, *it) = 1.0;
    } 
}