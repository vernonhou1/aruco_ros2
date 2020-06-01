

#include <string>
#include <memory>
#include <algorithm>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "rclcpp/rclcpp.hpp"


namespace aruco{

class ArucoDetc : public rclcpp::Node
{
public:
  ArucoDetc(const std::string &name, rclcpp::NodeOptions const &options);
  virtual ~ArucoDetc();

protected:
  std::atomic<bool> canceled_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_pub_;
  void imageCb(sensor_msgs::msg::Image::SharedPtr msg);
  // struct CameraMatrix {
  //   double fx;
  //   double fy;
  //   double cx;
  //   double cy;
  //   CameraMatrix(double fx, double fy, double cx, double cy)
  //     : 
  //       fx(fx),
  //       fy(fy),
  //       cx(cx),
  //       cy(cy) { }
  // };
  // struct DistCoeffs {
  //   double d0;
  //   double d1;
  //   double d2;
  //   double d3;
  //   DistCoeffs(double width, double height, double fx, double fy, double cx, double cy, double d0, double d1, double d2, double d3)
  //     : d0(d0),
  //       d1(d1),
  //       d2(d2),
  //       d3(d3) { }
  // };
  // CamearaMatrix cameraMatrix;
  // DistCoeffs distCoeffs;


} //namespace aruco