
#include <iostream>
#include <string>
#include <memory>
#include <algorithm>
#include "ros2_aruco/aruco_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/core.hpp>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace aruco{


ArucoDetc::ArucoDetc (const std::string &name, rclcpp::NodeOptions const &options)
  : rclcpp::Node(name, options),
  cam_param_(0, 0, 0, 0, 0, 0, 0, 0),
  canceled_(false){

  CreateParameter();

  cameraMatrix.create(3,3,CV_64FC1);
  cameraMatrix.setTo(0);
  cameraMatrix.at<double>(0,0) = cam_param_.fx;   cameraMatrix.at<double>(0,1) = 0;               cameraMatrix.at<double>(0,2) = cam_param_.cx;
  cameraMatrix.at<double>(1,0) = 0;               cameraMatrix.at<double>(1,1) = cam_param_.fy;   cameraMatrix.at<double>(1,2) = cam_param_.cy;
  cameraMatrix.at<double>(2,0) = 0;               cameraMatrix.at<double>(2,1) = 0;               cameraMatrix.at<double>(2,2) = 0;

  distorsionCoeff.create(4,1,CV_64FC1);
  distorsionCoeff.at<double>(0, 0) = cam_param_.d0;
  distorsionCoeff.at<double>(1, 0) = cam_param_.d1;
  distorsionCoeff.at<double>(2, 0) = cam_param_.d2;
  distorsionCoeff.at<double>(3, 0) = cam_param_.d3;
 
  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw",
     10, 
    std::bind(&ArucoDetc::imageCb,this,std::placeholders::_1));
  

//   int tvecsSize = tvecs.size();
//   for(int k=0; k<tvecsSize; k++){         
//                   std::cout << "t ->" <<std::endl;
//                   double x = tvecs[k].x;  
//                   double y = tvecs[k].y;   
//                   double z = tvecs[k].z; 
//                   std::cout << x << "  " << y << "  " <<z<<std::endl;
// }

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("/out_image", 10);

  
  }

  ArucoDetc::~ArucoDetc(){
    canceled_.store(true);
  }
  
  

  void ArucoDetc::CreateParameter() {
  cam_param_.fx = declare_parameter<double>("cam_fx", 1027.583197);
  cam_param_.fy = declare_parameter<double>("cam_fy", 1041.415347);
  cam_param_.cx = declare_parameter<double>("cam_cx", 0);
  cam_param_.cy = declare_parameter<double>("cam_cy", 0);
  cam_param_.d0 = declare_parameter<double>("cam_d0", 0);
  cam_param_.d1 = declare_parameter<double>("cam_d1", 0);
  cam_param_.d2 = declare_parameter<double>("cam_d2", 0);
  cam_param_.d3 = declare_parameter<double>("cam_d3", 0);

  set_on_parameters_set_callback(
      [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto const &p : parameters) {
          result.successful &= HandleParameter(p);
        }
        return result;
      });
  }

  bool ArucoDetc::HandleParameter(rclcpp::Parameter const &param) {
  if (param.get_name() == "cam_fx"){
    cam_param_.fx = param.as_double();
  }else if (param.get_name() == "cam_fy"){
    cam_param_.fy = param.as_double();
  }else if (param.get_name() == "cam_cx"){
    cam_param_.cx = param.as_double();
  }else if (param.get_name() == "cam_cy"){
    cam_param_.cy = param.as_double();
  }else if (param.get_name() == "cam_d0"){
    cam_param_.d0 = param.as_double();
  }else if (param.get_name() == "cam_d1"){
    cam_param_.d1 = param.as_double();
  }else if (param.get_name() == "cam_d2"){
    cam_param_.d2 = param.as_double();
  }else if (param.get_name() == "cam_d3"){
    cam_param_.d3 = param.as_double();
  }{
    return false;
  }
  return true;
}


  void ArucoDetc::imageCb(sensor_msgs::msg::Image::SharedPtr msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    auto outImg = std::make_unique<sensor_msgs::msg::Image>();
    
    cv::Mat img = cv_ptr -> image;

  
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::Mat outputImage = img.clone();
    
    if (markerIds.size() > 0) {
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distorsionCoeff, rvecs, tvecs);
        // draw axis for each marker
        for(int i=0; i<markerIds.size(); i++)
            cv::aruco::drawAxis(outputImage, cameraMatrix, distorsionCoeff, rvecs[i], tvecs[i], 0.1);
    }
    

    
    cv_ptr -> image =outputImage;
    cv_ptr ->toImageMsg(*outImg);
    image_pub_->publish(std::move(outImg));


    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs,rotation_matrix);
    std::stringstream r;
    r << rotation_matrix;
    // std::copy(rotation_matrix.begin(), rotation_matrix.end(), std::ostream_iterator<cv::Mat>(r, " "));
    RCLCPP_INFO(this->get_logger(), "R -> %s", r.str().c_str());

    std::stringstream t;
    std::copy(tvecs.begin(), tvecs.end(), std::ostream_iterator<cv::Vec3d>(t, " "));
    RCLCPP_INFO(this->get_logger(), "t -> %s", t.str().c_str());

  }

  
} //namespace aruco



