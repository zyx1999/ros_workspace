#include "grid_map_demos/SDFDescriptor.hpp"

void SDFDescriptor::toTXT(const std::vector<cv::Mat>& src, const std::vector<std::string>& pathnames){
    for(int i = 0; i <  pathnames.size(); i++){
        Eigen::Matrix<float, -1, -1> target;
        cv::cv2eigen(src[i], target);
        std::ofstream fout(pathnames[i]);
        if(fout.is_open()){
            fout<<target;
            fout.close();
            ROS_INFO("Matrix written to file [%s] successfully.", pathnames[i].c_str());
        } else{
            ROS_WARN("Unable to open file [%s]", pathnames[i].c_str());
        }
    }
}

void SDFDescriptor::find_extrema_points(const cv::Mat& src_sdf, const cv::Mat& src_doh, std::vector<cv::Point>& dst_extrema_points){
    // dst_extrema_full = cv::Mat::zeros(src_doh.size(), CV_32FC1);
    // loop through each pixel in the image
    cv::Mat extrema = cv::Mat::zeros(src_doh.size(), CV_8UC1);
    for(int i = 1; i < src_doh.rows - 1; i++){
        for(int j = 1; j < src_doh.cols - 1; j++){
            // check if the current pixel is an extremum
            float value = src_doh.at<float>(i, j);
            bool is_extremum = true;
            for(int k = -1; k <= 1; k++){
                for(int l = -1; l <= 1; l++){
                    if(value < src_doh.at<float>(i+k, j+l)){
                        is_extremum = false;
                        break;
                    }
                }
                if(!is_extremum){
                    break;
                }
            }
            if(is_extremum){
                // dst_extrema_full.at<float>(i, j) = src_sdf.at<float>(i, j);
                extrema.at<uchar>(i, j) = 255;
            }
        }
    }
    cv::findNonZero(extrema, dst_extrema_points);
}
/// @brief Classify extrema points by the sign of eigen value.
/// @param src_extrema_points 
/// @param src_eigenvalue1 
/// @param src_eigenvalue2 
/// @param dst 
void SDFDescriptor::classify_extrema_points(const std::vector<cv::Point>& src_extrema_points, 
                                            cv::Mat& src_eigenvalue1 , cv::Mat& src_eigenvalue2, 
                                            std::vector<std::vector<cv::Point>>& dst){
    dst = std::vector<std::vector<cv::Point>>(3);
    // 0: extrema max; 1: extrema min, 2: extrema saddle
    for(const auto& pt: src_extrema_points){
        float ev1 = src_eigenvalue1.at<float>(pt.x, pt.y);
        float ev2 = src_eigenvalue2.at<float>(pt.x, pt.y);
        if(ev1 < 0 && ev2 < 0){
            dst[0].push_back(pt);
        }
        if(ev1 > 0 && ev2 > 0){
            dst[1].push_back(pt);
        }
        if(ev1 * ev2 < 0){
            dst[2].push_back(pt);
        }
    }
}

void SDFDescriptor::detect_gaussian_curvature_and_eigen(const cv::Mat& src, int ksize, cv::Mat& dst_doh, cv::Mat& dst_eigenvalue1 , cv::Mat& dst_eigenvalue2){
    cv::Mat gaussBlur, dx, dy, dxx, dxy, dyy;
    // 1. gaussian blue
    cv::GaussianBlur(src, gaussBlur, cv::Size(5, 5), 0, 0);
    
    // 2. hessian
    cv::Sobel(gaussBlur, dx, -1, 1, 0, ksize);
    cv::Sobel(gaussBlur, dy, -1, 0, 1, ksize);
    cv::Sobel(dx, dxx, -1, 1, 0, ksize);
    cv::Sobel(dx, dxy, -1, 0, 1, ksize);
    cv::Sobel(dy, dyy, -1, 0, 1, ksize);

    // 3. DoH & gaussian curvature & eigen
    dst_doh.create(src.size(), CV_32FC1);
    dst_eigenvalue1.create(src.size(), CV_32FC1);
    dst_eigenvalue2.create(src.size(), CV_32FC1);

    for(int i = 0; i < src.rows; i++){
        for(int j = 0; j < src.cols; j++){
            float fx = dx.at<float>(i, j);
            float fy = dy.at<float>(i, j);
            float fxx = dxx.at<float>(i, j);
            float fxy = dxy.at<float>(i, j);
            float fyy = dyy.at<float>(i, j);
            // DoH
            float doh = fxx * fyy - fxy * fxy;
            // gaussian curvature
            // float k = doh / pow(fx * fx + fy * fy + 1e-8, 2);

            // eigen value & eigen vector
            cv::Mat hessianAtEachPoint_(2, 2, CV_32FC1);
            hessianAtEachPoint_.at<float>(0, 0) = fxx;
            hessianAtEachPoint_.at<float>(0, 1) = fxy;
            hessianAtEachPoint_.at<float>(1, 0) = fxy;
            hessianAtEachPoint_.at<float>(1, 1) = fyy;
            cv::Mat eigenValue_, eigenVector_;
            cv::eigen(hessianAtEachPoint_, eigenValue_, eigenVector_);
            // float gaussCurv_ = eigenValue_.at<float>(0, 0) * eigenValue_.at<float>(1, 0);
            // gaussianCurvature_.at<float>(i, j) = gaussCurv_;

            dst_doh.at<float>(i, j) = doh;
            dst_eigenvalue1.at<float>(i, j) = eigenValue_.at<float>(0, 0);
            dst_eigenvalue2.at<float>(i, j) = eigenValue_.at<float>(1, 0);

            // dst.at<float>(i, j) = k;
            // dst.at<float>(i, j) = gaussCurv_;
        }
    }
}

bool SDFDescriptor::srvCallback(grid_map_demos::sdfDetect::Request& req, grid_map_demos::sdfDetect::Response& res){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(req.sdf_map, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat src_sdf_ = cv_ptr->image;
    cv::Mat doh_, eigenValue1_, eigenValue2_;
    std::vector<cv::Point> extrema_points_;
    std::vector<std::vector<cv::Point>> classified_extrema_points_;

    detect_gaussian_curvature_and_eigen(src_sdf_, 3, doh_, eigenValue1_, eigenValue2_);
    find_extrema_points(src_sdf_, doh_, extrema_points_);
    classify_extrema_points(extrema_points_, eigenValue1_, eigenValue2_, classified_extrema_points_);

    sensor_msgs::PointCloud msg_extrema_points;
    int offset_ = 1;
    for(int i = 0; i < 3; i++){
        for(const auto& pt: classified_extrema_points_[i]){
            geometry_msgs::Point32 point32;
            point32.x = pt.x;
            point32.y = pt.y;
            point32.z = float(i+offset_);
            msg_extrema_points.points.push_back(point32);
        }
    }
    res.keypoints = msg_extrema_points;
    return true;
}


void SDFDescriptor::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        ROS_INFO_THROTTLE(1.0, "[Callback] SDF Descriptor...");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    cv::Mat src_sdf_ = cv_ptr->image;
    cv::Mat doh_, eigenValue1_, eigenValue2_;
    std::vector<cv::Point> extrema_points_;
    std::vector<std::vector<cv::Point>> classified_extrema_points_;

    detect_gaussian_curvature_and_eigen(src_sdf_, 3, doh_, eigenValue1_, eigenValue2_);
    find_extrema_points(src_sdf_, doh_, extrema_points_);
    classify_extrema_points(extrema_points_, eigenValue1_, eigenValue2_, classified_extrema_points_);

    sensor_msgs::PointCloud msg_extrema_points;
    int offset_ = 1;
    for(int i = 0; i < 3; i++){
        for(const auto& pt: classified_extrema_points_[i]){
            geometry_msgs::Point32 point32;
            point32.x = pt.x;
            point32.y = pt.y;
            point32.z = float(i+offset_);
            msg_extrema_points.points.push_back(point32);
        }
    }

    // for(const auto& pt : extrema_points_){
    //     geometry_msgs::Point32 point32;
    //     point32.x = pt.x;
    //     point32.y = pt.y;
    //     msg_extrema_points.points.push_back(point32);
    // }
    pub_extrema_points_.publish(msg_extrema_points);

    if(once_ == 0){
        once_++;
        std::vector<cv::Mat> data_{doh_};
        std::vector<std::string> filenames_{"/home/yuxuanzhao/Desktop/gaussCurv.txt"};
        toTXT(data_, filenames_);
    }
}
