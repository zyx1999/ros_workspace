#include "grid_map_demos/SDFDescriptor.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "detector");

    // cv::namedWindow("sdf message", CV_WINDOW_NORMAL);
    // cv::namedWindow("Gaussian Curvature", CV_WINDOW_NORMAL);
    // cv::startWindowThread();
    SDFDescriptor sdfdetector;
    
    ros::spin();

    // cv::destroyAllWindows();
    return 0;
}