

#include <string>
#include <memory>
#include <algorithm>
#include "aruco_det/aruco_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

namespace aruco{


ArucoDetc::ArucoDetc (const std::string &name, rclcpp::NodeOptions const &options)
  : rclcpp::Node(name, options),
  canceled_(false){
 
  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw",
     10, 
    std::bind(&ArucoDetc::imageCb,this,std::placeholders::_1));

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("/out_image", 10);
  }

  ArucoDetc::~ArucoDetc(){
    canceled_.store(true);
  }

  void ArucoDetc::imageCb(sensor_msgs::msg::Image::SharedPtr msg) {
    auto cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    auto outImg = std::make_unique<sensor_msgs::msg::Image>();
    
    cv::Mat img = cv_ptr -> image;


    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat outputImage = img.clone();
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(img, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
        // draw axis for each marker
        for(int i=0; i<ids.size(); i++)
            cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    }

    // cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds);
    
    
    // cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    
    
    
    cv_ptr -> image =outputImage;
    cv_ptr ->toImageMsg(*outImg);
    image_pub_->publish(std::move(outImg));

  }

  
} //namespace aruco



