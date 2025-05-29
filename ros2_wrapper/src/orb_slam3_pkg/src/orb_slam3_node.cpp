#include <GL/glew.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <ORB_SLAM3_ROS2/System.h>

class OrbSlam3Node : public rclcpp::Node
{
public:
  OrbSlam3Node()
  : Node("orb_slam3_node")
  {
    // Use SensorDataQoS for cameras
    auto qos = rclcpp::SensorDataQoS();

    // 1) Compressed image via image_transport
    m_compressed_img_sub = image_transport::create_subscription(
      this,
      "/camera/image_raw",
      std::bind(&OrbSlam3Node::compressed_img_callback,
                this, std::placeholders::_1),
      "compressed",
      qos.get_rmw_qos_profile());

    // 2) Raw image via rclcpp
    m_raw_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      qos,  // Note: pass the QoS object directly
      std::bind(&OrbSlam3Node::raw_img_callback,
                this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3 node started");
  }

private:
  // Must take ConstSharedPtr for both
  void compressed_img_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try {
      cv::Mat frame =
        cv_bridge::toCvShare(msg, "bgr8")->image;
      // … process with ORB_SLAM3 …
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "cv_bridge exception: %s", e.what());
    }
  }

  void raw_img_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    RCLCPP_INFO(get_logger(),
                "Got raw image %ux%u",
                msg->width, msg->height);
    // … later: feed into ORB_SLAM3 System …
  }

  image_transport::Subscriber
    m_compressed_img_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    m_raw_img_sub;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrbSlam3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
