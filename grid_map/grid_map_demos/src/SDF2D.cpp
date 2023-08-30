#include "grid_map_demos/SDF2D.hpp"


SDF2D::SDF2D(ros::NodeHandle& nh): nh_(nh){
    ros::service::waitForService("/img2PC");
    client_img2PC = nh_.serviceClient<grid_map_demos::img2PointCloud>("/img2PC");
    client_sdf = nh_.serviceClient<grid_map_demos::sdfDetect>("/sdf_service");
    publisher = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    std::shared_ptr<SingleMap> ptr_map1, ptr_map2, ptr_combine;
    mapFromImage(ptr_map1);
    mapFromImage(ptr_map2);
    SDFAlign(ptr_map1, ptr_map2);
    combineTwoMap(ptr_map1, ptr_map2, ptr_combine);
    setDisplayMap(1);
}

std::vector<cv::KeyPoint> convertMatToKeyPoints(const cv::Mat& mat) {
    std::vector<cv::KeyPoint> keypoints;
    // Check if the input matrix is of the right size and type
    if (mat.empty() || mat.cols != 3 || mat.type() != CV_32F) {
        return keypoints; // Return an empty vector
    }
    for (int i = 0; i < mat.rows; i++) {
        float x = mat.at<float>(i, 0);
        float y = mat.at<float>(i, 1);
        float response = mat.at<float>(i, 2);

        // Assuming the third column is the response
        keypoints.push_back(cv::KeyPoint(cv::Point2f(x, y), 1 /* size */, -1 /* angle */, response));
    }
    return keypoints;
}

void msgToMat(sensor_msgs::Image msg, cv::Mat& cv_image){
    sensor_msgs::ImageConstPtr img_ptr = boost::make_shared<sensor_msgs::Image const>(msg);
    cv_image = cv_bridge::toCvShare(img_ptr, "bgr8")->image;  
}
void savedImage(sensor_msgs::Image& msg, std::string saved_path){
    cv::Mat cv_image;
    msgToMat(msg, cv_image);
    cv::imwrite(saved_path, cv_image);
}
void ORBAlign(sensor_msgs::Image msg1, sensor_msgs::Image msg2){
    cv::Mat image1, image2;

    msgToMat(msg1, image1);
    cv::imwrite("/home/yuxuanzhao/Desktop/image1.jpg", image1);

    msgToMat(msg2, image2);
    cv::imwrite("/home/yuxuanzhao/Desktop/image2.jpg", image2);

    // 初始化ORB检测器
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    // 检测关键点和计算描述子
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    orb->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);

    // 使用BFMatcher进行匹配
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // 仅选择最佳匹配
    std::sort(matches.begin(), matches.end());
    const int numGoodMatches = matches.size() * 0.2;
    matches.erase(matches.begin() + numGoodMatches, matches.end());

    // Draw top matches
    cv::Mat imMatches;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, imMatches);
    cv::imwrite("/home/yuxuanzhao/Desktop/matches.jpg", imMatches);

    // 使用Homo
    std::vector<cv::Point2f> points1, points2;

    for (size_t i = 0; i < matches.size(); i++) {
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    cv::Mat homo = cv::findHomography(points1, points2, cv::RANSAC);
    cv::Mat result;

    cv::warpPerspective(image1, result, homo, image1.size());
    cv::imwrite("/home/yuxuanzhao/Desktop/result.jpg", result);
}
void SDF2D::SDFAlign(std::shared_ptr<SingleMap>& ptr_map1, std::shared_ptr<SingleMap>& ptr_map2){
    cv::Mat descriptors1, descriptors2;
    ptr_map1->descriptors.convertTo(descriptors1, CV_32F);
    ptr_map2->descriptors.convertTo(descriptors2, CV_32F);

    cv::Mat image1, image2;
    msgToMat(ptr_map1->img, image1);
    cv::imwrite("/home/yuxuanzhao/Desktop/map1_img.jpg", image1);
    msgToMat(ptr_map2->img, image2);
    cv::imwrite("/home/yuxuanzhao/Desktop/map2_img.jpg", image2);
    // cv::eigen2cv(ptr_map1->map.get("sdf2d"), out_sdf);
    // cv::imwrite("/home/yuxuanzhao/Desktop/map1_sdf.jpg", out_sdf);
    
    // 使用BFMatcher进行匹配
    cv::BFMatcher matcher(cv::NORM_L2); // Use L2 norm for matching histograms
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    // 仅选择最佳匹配
    std::sort(matches.begin(), matches.end());
    const int numGoodMatches = matches.size() * 0.06;
    matches.erase(matches.begin() + numGoodMatches, matches.end());

    // for(const auto& match : matches) {
    //     ROS_INFO("Query index: %d Train index: %d Distance: %f", 
    //         match.queryIdx, match.trainIdx, match.distance);
    // } 

    // Draw top matches
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    keypoints1 = convertMatToKeyPoints(ptr_map1->keypoints);
    keypoints2 = convertMatToKeyPoints(ptr_map2->keypoints);
    cv::Mat imMatches;
    cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, imMatches);
    cv::imwrite("/home/yuxuanzhao/Desktop/matches.jpg", imMatches);

    // 使用Homo
    std::vector<cv::Point2f> points1, points2;

    for (size_t i = 0; i < matches.size(); i++) {
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    cv::Mat homo = cv::findHomography(points1, points2, cv::RANSAC);
    cv::Mat result;

    cv::warpPerspective(image1, result, homo, image1.size());
    cv::imwrite("/home/yuxuanzhao/Desktop/result.jpg", result);
}

cv::Mat convertPointCloud20ToMat(const grid_map_demos::PointCloud20& cloud) {
    // 创建一个空的Mat，行数与点云中的点数相同，每行有20个元素
    cv::Mat mat(cloud.points.size(), 20, CV_32F);

    for (size_t i = 0; i < cloud.points.size(); i++) {
        const auto& point = cloud.points[i];
        // 假设Point20消息的成员是point1, point2,..., point20
        mat.at<float>(i, 0) = point.x;
        mat.at<float>(i, 1) = point.y;
        mat.at<float>(i, 2) = point.type;
        mat.at<float>(i, 3) = point.hist1;
        mat.at<float>(i, 4) = point.hist2;
        mat.at<float>(i, 5) = point.hist3;
        mat.at<float>(i, 6) = point.hist4;
        mat.at<float>(i, 7) = point.hist5;
        mat.at<float>(i, 8) = point.hist6;
        mat.at<float>(i, 9) = point.hist7;
        mat.at<float>(i, 10) = point.hist8;
        mat.at<float>(i, 11) = point.hist9;
        mat.at<float>(i, 12) = point.hist10;
        mat.at<float>(i, 13) = point.hist11;
        mat.at<float>(i, 14) = point.hist12;
        mat.at<float>(i, 15) = point.hist13;
        mat.at<float>(i, 16) = point.hist14;
        mat.at<float>(i, 17) = point.hist15;
        mat.at<float>(i, 18) = point.hist16;
        mat.at<float>(i, 19) = point.hist17;
    }
    return mat;
}

void SDF2D::combineTwoMap(std::shared_ptr<SingleMap>& ptr_map1, std::shared_ptr<SingleMap>& ptr_map2, std::shared_ptr<SingleMap>& ptr_out){
    ptr_out = std::make_shared<SingleMap>();
    ptr_out->map.setFrameId("map");

    float multiple_ = 1/ptr_map1->map_resolution_;
    float border_length = 4;
    grid_map::Length out_length(ptr_out->map_length_(0), ptr_out->map_length_(1)*2+border_length);
    ptr_out->map.setGeometry(out_length, ptr_map1->map_resolution_, ptr_map1->map_position_);

    // elevation layer
    Eigen::MatrixXf mat1 = ptr_map1->map.get("elevation");
    Eigen::MatrixXf mat2 = ptr_map2->map.get("elevation");
    Eigen::MatrixXf border(mat1.rows(), int(multiple_*border_length));
    border.setZero();
    Eigen::MatrixXf hcat(mat1.rows(), mat1.cols() + border.cols() +mat2.cols());
    hcat << mat1, border, mat2;
    ptr_out->map.add("elevation", hcat);

    // sdf2d layer 
    mat1 = ptr_map1->map.get("sdf2d");
    mat2 = ptr_map2->map.get("sdf2d");
    Eigen::MatrixXf hcatSDF(mat1.rows(), mat1.cols() + border.cols() +mat2.cols());
    hcatSDF << mat1, border, mat2;
    ptr_out->map.add("sdf2d", hcatSDF);

    Eigen::Array2i ar = ptr_out->map.getSize();
    ROS_INFO("ptr_out->map.getSize(): (%d, %d)", ar(0), ar(1));
    Eigen::Array2i ar1 = ptr_map1->map.getSize();
    ROS_INFO("ptr_map1->map.getSize(): (%d, %d)", ar1(0), ar1(1));
    ROS_INFO("Mat1: %ld x %ld", mat1.rows(), mat1.cols());
    
    ptrs.push_back(ptr_out);
}
bool SDF2D::setDisplayMap(int idx){
    displayMap = ptrs[idx]->map;
}
void SDF2D::displayKeypoints(cv::Mat& kpAndDesc, SingleMap& sgmap_){
    int type_offset = 1;
    cv::Mat data_extrema_max = cv::Mat::zeros(sgmap_.rows_, sgmap_.cols_, CV_32FC1);
    cv::Mat data_extrema_min = cv::Mat::zeros(sgmap_.rows_, sgmap_.cols_, CV_32FC1);
    cv::Mat data_extrema_saddle = cv::Mat::zeros(sgmap_.rows_, sgmap_.cols_, CV_32FC1);
    for(int row = 0; row < kpAndDesc.rows; row++){
        float* ptr = kpAndDesc.ptr<float>(row);
        float value = sgmap_.map.at("sdf2d", grid_map::Index(ptr[0], ptr[1]));
        if(ptr[2] - type_offset == 0){
            data_extrema_max.at<float>(ptr[0], ptr[1]) = value;
        }
        if(ptr[2] - type_offset == 1){
            data_extrema_min.at<float>(ptr[0], ptr[1]) = value;
        }
        if(ptr[2] - type_offset == 2){
            data_extrema_saddle.at<float>(ptr[0], ptr[1]) = value;
        }
    }
    Eigen::Matrix<float, -1, -1> layer_extrema_max, layer_extrema_min, layer_extrema_saddle;
    cv::cv2eigen(data_extrema_max, layer_extrema_max);
    cv::cv2eigen(data_extrema_min, layer_extrema_min);
    cv::cv2eigen(data_extrema_saddle, layer_extrema_saddle);
    sgmap_.map.add("extrema_max", layer_extrema_max);
    sgmap_.map.add("extrema_min", layer_extrema_min);
    sgmap_.map.add("extrema_saddle", layer_extrema_saddle);
}

void SDF2D::mapFromImage(std::shared_ptr<SingleMap>& ptr){
    ptr = std::make_shared<SingleMap>();
    // 1. request image data from img2PC server.
    srv_img2PC.request.type = 1.0;
    client_img2PC.call(srv_img2PC);
    ptr->img = srv_img2PC.response.img;
    // convert sensor_msgs::Image to grid_map
    ptr->map.add(ptr->elevationLayer_);
    ptr->map.setFrameId("map");
    grid_map::GridMapRosConverter::initializeFromImage(ptr->img, ptr->map_resolution_, ptr->map);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", ptr->map.getLength().x(),
             ptr->map.getLength().y(), ptr->map.getSize()(0), ptr->map.getSize()(1));
    grid_map::GridMapRosConverter::addLayerFromImage(ptr->img, ptr->elevationLayer_, ptr->map, minHeight, maxHeight);
    ptr->rows_ = ptr->map.getSize()(0);
    ptr->cols_ = ptr->map.getSize()(1);

    // 2. Generate 2D SDF.
    auto& elevationData = ptr->map.get(ptr->elevationLayer_);
    if (elevationData.hasNaN()) {
        const float inpaint{elevationData.minCoeffOfFinites()};
        ROS_WARN("[SdfDemo] Map contains NaN values. Will apply inpainting with min value.");
        elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v)? v : inpaint; });
    }
    Eigen::Matrix<bool, -1, -1> occupancy = elevationData.unaryExpr([=](float val) { return val > 0.5; });
    grid_map::Matrix signedDistance = grid_map::signed_distance_field::signedDistanceFromOccupancy(occupancy, ptr->map_resolution_);
    ptr->map.add("sdf2d", signedDistance);

    // 3. get keypoints & descriptors from detector server.
    cv::Mat signedDistanceMat_;
    cv::eigen2cv(signedDistance, signedDistanceMat_);
    ros::service::waitForService("/sdf_service");
    sensor_msgs::ImagePtr msg_sdf = cv_bridge::CvImage(std_msgs::Header(), 
        sensor_msgs::image_encodings::TYPE_32FC1, signedDistanceMat_).toImageMsg();
    srv_sdf.request.sdf_map = *msg_sdf;
    client_sdf.call(srv_sdf);
    grid_map_demos::PointCloud20 data = srv_sdf.response.cloud;
    cv::Mat keypointsAndDescriptors = convertPointCloud20ToMat(data);

    // show keypoints
    displayKeypoints(keypointsAndDescriptors, *ptr);

    cv::Mat& kdmat = keypointsAndDescriptors;
    // 4. split 分割为两个子矩阵
    ptr->keypoints = kdmat(cv::Range::all(), cv::Range(0, 3)).clone(); // n x 3
    ptr->descriptors = kdmat(cv::Range::all(), cv::Range(3, kdmat.cols)).clone(); // n x 17

    ptrs.push_back(ptr);
    // Eigen::Matrix<float, -1, -1> target;
    // cv::cv2eigen(keypointsAndDescriptors, target);
    // std::string filename("/home/yuxuanzhao/Desktop/dpAndDesc.txt");
    // std::ofstream fout(filename);
    // if(fout.is_open()){
    //     fout<<target;
    //     fout.close();
    //     ROS_INFO("Matrix written to file [%s] successfully.", filename.c_str());
    // } else{
    //     ROS_WARN("Unable to open file [%s]", filename.c_str());
    // }
}