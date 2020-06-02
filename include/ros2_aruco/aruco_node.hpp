

#include <string>
#include <memory>
#include <algorithm>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rclcpp/rclcpp.hpp"
#include "opencv2/core.hpp"


namespace aruco{

class ArucoDetc : public rclcpp::Node
{
public:
  ArucoDetc(const std::string &name, rclcpp::NodeOptions const &options);
  virtual ~ArucoDetc();

protected:
  std::atomic<bool> canceled_;
  void CreateParameter();
  bool HandleParameter(rclcpp::Parameter const &param);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
  void imageCb(sensor_msgs::msg::Image::SharedPtr msg);
  // cv::Mat cameraMatrix(int x, int y, cv::CV_64FC1);
  // cv::Mat distorsionCoeff(int x, int y, cv::CV_64FC1);
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::Mat cameraMatrix;
  cv::Mat distorsionCoeff;
  int block_size_;
  double offset_value_;
  struct CamParam {
    double fx;
    double fy;
    double cx;
    double cy;
    double d0;
    double d1;
    double d2;
    double d3;
    CamParam( double fx, double fy, double cx, double cy, double d0, double d1, double d2, double d3)
      : fx(fx),
        fy(fy),
        cx(cx),
        cy(cy),
        d0(d0),
        d1(d1),
        d2(d2),
        d3(d3) { }
  };
  CamParam cam_param_;
};
} //namespace aruco