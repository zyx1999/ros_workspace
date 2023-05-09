#include "grid_map_demos/SDFDescriptor.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "detector");

    ros::NodeHandle nh("~");

    SDFDescriptor sdfdetector(nh);
    
    ros::spin();
    return 0;
}