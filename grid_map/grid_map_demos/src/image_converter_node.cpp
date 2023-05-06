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
int once = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        ROS_INFO_THROTTLE(1.0, "Callback...");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ;
    }
    // Eigen::Matrix<float, -1, -1> signedDistanceMatrix_;
    // cv::cv2eigen(cv_ptr->image, signedDistanceMatrix_);
    cv::Mat sdfSrc_ = cv_ptr->image;
    cv::Mat sdfBlur_, sdfDst_x_, sdfDst_y_, sdfDst_xx_, sdfDst_yy_, sdfDst_xy_;

    // 1. Gaussian blur
    cv::GaussianBlur(sdfSrc_, sdfBlur_, cv::Size(5, 5), 0, 0);

    // 2. Hessian: H_xx, H_yy, H_xy, H_yx(is equal to H_xy)
    cv::Sobel(sdfBlur_, sdfDst_x_, -1, 1, 0, 3);
    cv::Sobel(sdfBlur_, sdfDst_y_, -1, 0, 1, 3);
    cv::Sobel(sdfDst_x_, sdfDst_xx_, -1, 1, 0, 3);
    cv::Sobel(sdfDst_y_, sdfDst_yy_, -1, 0, 1, 3);
    cv::Sobel(sdfDst_x_, sdfDst_xy_, -1, 0, 1, 3);

    // 3. DoH = H_xx * H_yy - H_xy^2
    cv::Mat sdfDoH_, sdfXXTimesYY_, sdfPowHessianXY_;
    cv::multiply(sdfDst_xx_, sdfDst_yy_, sdfXXTimesYY_);
    cv::pow(sdfDst_xy_, 2.0, sdfPowHessianXY_);
    cv::subtract(sdfXXTimesYY_, sdfPowHessianXY_, sdfDoH_);

    // 4. eigen value & eigen vector of Hessian
    int sdfRows_{sdfDst_xx_.rows}, sdfCols_{sdfDst_xx_.cols};
    cv::Mat gaussianCurvature_(sdfRows_, sdfCols_, CV_32FC1);
    for(int i = 0; i < sdfRows_; i++){
        for(int j = 0; j < sdfCols_; j++){
            auto h_xx = sdfDst_xx_.at<float>(i, j);
            auto h_yy = sdfDst_yy_.at<float>(i, j);
            auto h_xy = sdfDst_xy_.at<float>(i, j);

            cv::Mat hessianAtEachPoint_(2, 2, CV_32FC1);
            hessianAtEachPoint_.at<float>(0, 0) = h_xx;
            hessianAtEachPoint_.at<float>(0, 1) = h_xy;
            hessianAtEachPoint_.at<float>(1, 0) = h_xy;
            hessianAtEachPoint_.at<float>(1, 1) = h_yy;

            cv::Mat eigenValue_, eigenVector_;
            cv::eigen(hessianAtEachPoint_, eigenValue_, eigenVector_);
            ROS_INFO_THROTTLE(1.0, "size of Hessian's eigen value: rows: %d, cols: %d .", eigenValue_.rows, eigenValue_.cols);
            float gaussCurv_ = eigenValue_.at<float>(0, 0) * eigenValue_.at<float>(1, 0);
            gaussianCurvature_.at<float>(i, j) = gaussCurv_;
        }
    }

    // cv::Mat img_rgb;
    // img_rgb = cv_ptr->image;
    cv::imshow("origin", sdfSrc_);
    cv::imshow("Gaussian Blur", sdfBlur_);
    cv::imshow("Sobel_X", sdfDst_x_);
    cv::imshow("Sobel_Y", sdfDst_y_);
    cv::imshow("Sobel_XX", sdfDst_xx_);
    cv::imshow("Sobel_YY", sdfDst_yy_);
    cv::imshow("Sobel_XY", sdfDst_xy_);
    cv::imshow("DoH", sdfDoH_);
    cv::imshow("Gaussian Curvature", gaussianCurvature_);


    if(once == 0){
        Eigen::Matrix<float, -1, -1> DoH_;
        cv::cv2eigen(sdfDoH_, DoH_);

        Eigen::Matrix<float, -1, -1> GC_;
        cv::cv2eigen(gaussianCurvature_, GC_); 
        
        once++;
        std::ofstream fout("/home/yuxuanzhao/Desktop/DoH.txt");
        fout<<DoH_;
        fout.close();

        std::ofstream fout2("/home/yuxuanzhao/Desktop/GC.txt");
        fout2<<GC_;
        fout2.close();
        ROS_INFO_THROTTLE(1.0, "Write doh...");
    }

    cv::waitKey(3);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh("~");

    cv::namedWindow("origin", CV_WINDOW_NORMAL);
    cv::namedWindow("Gaussian Blur", CV_WINDOW_NORMAL);
    cv::namedWindow("Sobel_X", CV_WINDOW_NORMAL);
    cv::namedWindow("Sobel_Y", CV_WINDOW_NORMAL);
    cv::namedWindow("Sobel_XX", CV_WINDOW_NORMAL);
    cv::namedWindow("Sobel_YY", CV_WINDOW_NORMAL);
    cv::namedWindow("Sobel_XY", CV_WINDOW_NORMAL);
    cv::namedWindow("DoH", CV_WINDOW_NORMAL);
    cv::namedWindow("Gaussian Curvature", CV_WINDOW_NORMAL);

    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/my_sdf_demo/signed_distance", 1, imageCallback);
    
    ros::spin();

    cv::destroyAllWindows();
    return 0;
}