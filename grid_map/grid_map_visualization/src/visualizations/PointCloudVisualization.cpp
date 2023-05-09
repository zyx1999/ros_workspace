/*
 * PointCloudVisualization.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_visualization/visualizations/PointCloudVisualization.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/PointCloud2.h>

namespace grid_map_visualization {

PointCloudVisualization::PointCloudVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
    : VisualizationBase(nodeHandle, name)
{
}

PointCloudVisualization::~PointCloudVisualization()
{
}

bool PointCloudVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);
  if (!getParam("layer", layer_)) {
    ROS_ERROR("PointCloudVisualization with name '%s' did not find a 'layer' parameter.", name_.c_str());
    return false;
  }
  if (!getParam("filterZero", filterZero_)){
    ROS_ERROR("PointCloudVisualization with name '%s' did not find a 'filterZero' parameter.", name_.c_str());
  }
  return true;
}

bool PointCloudVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(name_, 1, true);
  return true;
}

bool PointCloudVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  if (!map.exists(layer_)) {
    ROS_WARN_STREAM("PointCloudVisualization::visualize: No grid map layer with name '" << layer_ << "' found.");
    return false;
  }
  sensor_msgs::PointCloud2 msg_cloud;
  grid_map::GridMapRosConverter::toPointCloud(map, layer_, msg_cloud);

  // only save the keypoints with z-axis != 0.
  // Since we cannot modify the msg, therefore we should convert ros msg into pcl::PointCloud to filter the points
  // then turn the pcl::PointCloud back to ros msg.
  if(filterZero_){
    ROS_INFO("filterZero=[TRUE]");
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_tmp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg_cloud, *pcl_cloud_ptr);
    for(const auto& pt: *pcl_cloud_ptr){
      if(pt.z != 0){
        pcl_cloud_tmp_ptr->push_back(pt);
      }
    }
    pcl_cloud_ptr->points.clear();
    for(const auto& pt: *pcl_cloud_tmp_ptr){
      pcl_cloud_ptr->push_back(pt);
    }
    sensor_msgs::PointCloud2 msg_filtered_cloud;
    pcl::toROSMsg(*pcl_cloud_ptr, msg_filtered_cloud);
    publisher_.publish(msg_filtered_cloud);
  } else {
    publisher_.publish(msg_cloud);
  }
  return true;
}

} /* namespace */
