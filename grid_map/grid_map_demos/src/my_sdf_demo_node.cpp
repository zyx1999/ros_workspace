#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud.h>
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
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
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
        rows_ = map.getSize()(0);
        cols_ = map.getSize()(1);
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
        // imgTransSub = imgTrans.subscribe("extrema_points", 1, boost::bind(&SDF2D::callback, this, _1));

        sub_extrema_points_ = nh.subscribe<sensor_msgs::PointCloud>("extrema_points", 10, boost::bind(&SDF2D::callback, this, _1));
    }
    void generateSampleGridMap(grid_map::GridMap&, std::string&);
    void callback(const sensor_msgs::PointCloud::ConstPtr& msg);
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
    ros::Subscriber sub_extrema_points_;
    int rows_{0}, cols_{0};
};


int main(int argc, char **argv){
    ros::init(argc, argv, "my_sdf_demo");

    SDF2D sdf2d;

    ros::Rate rate(10.0);
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
void SDF2D::callback(const sensor_msgs::PointCloud::ConstPtr& msg){
    ROS_INFO("[Callback] SDF2D...");
    int offset_ = 1;
    cv::Mat data_extrema_max = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    cv::Mat data_extrema_min = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    cv::Mat data_extrema_saddle = cv::Mat::zeros(rows_, cols_, CV_32FC1);
    for(const auto& pt : msg->points){
        float value = map.at("sdf2d", Index(pt.x, pt.y));
        if(pt.z - offset_ == 0){
            data_extrema_max.at<float>(pt.x, pt.y) = value;
        }
        if(pt.z - offset_ == 1){
            data_extrema_min.at<float>(pt.x, pt.y) = value;
        }
        if(pt.z - offset_ == 2){
            data_extrema_saddle.at<float>(pt.x, pt.y) = value;
        }
    }
    Eigen::Matrix<float, -1, -1> layer_extrema_max, layer_extrema_min, layer_extrema_saddle;
    cv::cv2eigen(data_extrema_max, layer_extrema_max);
    cv::cv2eigen(data_extrema_min, layer_extrema_min);
    cv::cv2eigen(data_extrema_saddle, layer_extrema_saddle);
    map.add("extrema_max", layer_extrema_max);
    map.add("extrema_min", layer_extrema_min);
    map.add("extrema_saddle", layer_extrema_saddle);
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