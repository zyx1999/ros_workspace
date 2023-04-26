/*
 * grid_map_visualization_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "grid_map_visualization/GridMapVisualization.hpp"

int main(int argc, char** argv)
{
  // Initialize a node named grid_map_visualization
  ros::init(argc, argv, "grid_map_visualization");
  // create a NodeHandle with namespace "~"
  ros::NodeHandle nodeHandle("~");
  /*
  * what constructor do in GridMapVisualization
  * 1. read parameters
  *   1.1 capture grid_map topic from param-server.
  *   1.2 capture visualization-config from param-server.
  *   1.3 check visualization-confg.
  *   1.4 create Visualization instance according to v-config. 
  * 2. create a timer of NodeHandle
  * 
  * 3. initialization
  *   3.1 iterator through all visualization instance to advertise corresponding topic
  *   3.2 subscribe topic: /grid_map_tutorial_demo/grid_map
  *   3.3 callback function: 
  *       a. convert topic message to grid map
  *       b. iterator through all visualization instance to visualize(publish) map.
  */
  grid_map_visualization::GridMapVisualization gridMapVisualization(nodeHandle, "grid_map_visualizations");

  ros::spin();
  return 0;
}
