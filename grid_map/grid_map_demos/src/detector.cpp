#include "grid_map_demos/SDFServer.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "detector");

    ros::NodeHandle nh("~");

    SDFServer sdfServer(nh);
    
    ros::spin();
    return 0;
}