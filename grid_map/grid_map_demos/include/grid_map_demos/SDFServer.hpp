#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "grid_map_demos/sdfDetect.h"
#include "grid_map_demos/PointCloud17.h"
#include "grid_map_demos/Point17.h"
class SDFKeyPoint;
class SDFServer{
public:
    SDFServer(ros::NodeHandle& nh): nh_(nh), it_(nh_){
        // Now use C/S to pass sdf map
        service_ = nh_.advertiseService("/sdf_service", &SDFServer::srvCallback, this);
    }
    bool srvCallback(grid_map_demos::sdfDetect::Request& req, grid_map_demos::sdfDetect::Response& res);
    void imageCallback(const sensor_msgs::ImageConstPtr&);
    void toTXT(const std::vector<cv::Mat>&, const std::vector<std::string>&);
    void detect_gaussian_curvature_and_eigen(const cv::Mat& src, int ksize, cv::Mat& dst_doh, cv::Mat& dst_eigenvalue1 , cv::Mat& dst_eigenvalue2);
    void find_extrema_points(const cv::Mat& src_doh, std::vector<cv::Point>& dst_extrema_points);
    void classify_extrema_points(const std::vector<cv::Point>& src_extrema_points, cv::Mat& src_eigenvalue1 , cv::Mat& src_eigenvalue2, std::vector<std::vector<cv::Point>>& dst);
    void makeDescriptorForSingleKeypoint(cv::Mat& src_sdf_, cv::Point& keypoint, int point_type);
    float gaussianDistanceWeight(int i, int j);

private:
    int once_{0};
    int radius_{10};
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Publisher pub_extrema_points_;
    image_transport::ImageTransport it_;
    image_transport::Publisher ipub_;
    image_transport::Subscriber isub_;
    std::vector<std::vector<SDFKeyPoint>> sdfkeypoints_ = std::vector<std::vector<SDFKeyPoint>>(4);
};

class SDFKeyPoint{
public:
    SDFKeyPoint(cv::Point& cord, float avg_sdf, std::vector<float>& hist_17bin, int type)
        : cord_(cord), avg_sdf_(avg_sdf), hist_17bin_(hist_17bin), type_(type){}
    cv::Point cord_;
    float avg_sdf_;
    std::vector<float> hist_17bin_;
    // 0: maximal, 1: minimal, 2: saddle, 3: critical, 4: unknown
    int type_;
};