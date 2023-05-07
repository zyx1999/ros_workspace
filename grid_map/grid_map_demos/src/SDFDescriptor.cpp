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

void SDFDescriptor::find_extrema_points(cv::Mat& sdf, cv::Mat& gauss, std::vector<cv::Point>& extrema_points, cv::Mat& extrema){
    extrema = cv::Mat::zeros(gauss.size(), CV_32FC1);
    // loop through each pixel in the image
    for(int i = 1; i < gauss.rows - 1; i++){
        for(int j = 1; j < gauss.cols - 1; j++){
            // check if the current pixel is an extremum
            float value = gauss.at<float>(i, j);
            bool is_extremum = true;
            for(int k = -1; k <= 1; k++){
                for(int l = -1; l <= 1; l++){
                    if(value < gauss.at<float>(i+k, j+l)){
                        is_extremum = false;
                        break;
                    }
                }
                if(!is_extremum){
                    break;
                }
            }
            if(is_extremum){
                extrema.at<float>(i, j) = sdf.at<float>(i, j);
            }
        }
    }
    cv::findNonZero(extrema, extrema_points);
}

void SDFDescriptor::calculate_gaussian_curvature(const cv::Mat& src, int ksize, cv::Mat& dst){
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
    dst.create(src.size(), CV_32FC1);
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
            float k = doh / pow(fx * fx + fy * fy + 1e-8, 2);

            // eigen value & eigen vector
            cv::Mat hessianAtEachPoint_(2, 2, CV_32FC1);
            hessianAtEachPoint_.at<float>(0, 0) = fxx;
            hessianAtEachPoint_.at<float>(0, 1) = fxy;
            hessianAtEachPoint_.at<float>(1, 0) = fxy;
            hessianAtEachPoint_.at<float>(1, 1) = fyy;
            cv::Mat eigenValue_, eigenVector_;
            cv::eigen(hessianAtEachPoint_, eigenValue_, eigenVector_);
            float gaussCurv_ = eigenValue_.at<float>(0, 0) * eigenValue_.at<float>(1, 0);
            // gaussianCurvature_.at<float>(i, j) = gaussCurv_;

            dst.at<float>(i, j) = k;
            // dst.at<float>(i, j) = gaussCurv_;
        }
    }
}

void SDFDescriptor::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        ROS_INFO_THROTTLE(1.0, "Callback...");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }

    cv::Mat sdfSrc_ = cv_ptr->image;
    cv::Mat gaussCurv_, extrema_;
    std::vector<cv::Point> extrema_points_;

    calculate_gaussian_curvature(sdfSrc_, 3, gaussCurv_);
    find_extrema_points(sdfSrc_, gaussCurv_, extrema_points_, extrema_);

    sensor_msgs::ImagePtr msg_extrema_points;
    msg_extrema_points = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, extrema_).toImageMsg();
    // publish back to map node
    ipub_.publish(msg_extrema_points);

    // cv::imshow("sdf message", sdfSrc_);
    // cv::imshow("Gaussian Curvature", gaussCurv_);
    // cv::waitKey(3);

    if(once_ == 0){
        once_++;
        std::vector<cv::Mat> data_{gaussCurv_, extrema_};
        std::vector<std::string> filenames_{"/home/yuxuanzhao/Desktop/gaussCurv.txt", "/home/yuxuanzhao/Desktop/extrema.txt"};
        toTXT(data_, filenames_);
    }
}
