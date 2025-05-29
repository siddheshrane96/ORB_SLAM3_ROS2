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
    
    this->declare_parameter<std::string>("vocab_path", "");
    this->declare_parameter<std::string>("settings_path", "");
    this->get_parameter("vocab_path", m_vocab_path);
    this->get_parameter("settings_path", m_setting_path);
    this->get_parameter("image_topic", m_image_topic);

    m_slam = std::make_unique<ORB_SLAM3::System>(
      m_vocab_path,
      m_setting_path,
      ORB_SLAM3::System::MONOCULAR,
      true);
    
    auto qos = rclcpp::SensorDataQoS();

    m_compressed_img_sub = image_transport::create_subscription(
      this,
      "/camera/image_raw",
      std::bind(&OrbSlam3Node::compressed_img_callback,
                this, std::placeholders::_1),
      "compressed",
      qos.get_rmw_qos_profile());

    m_raw_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      qos,  // Note: pass the QoS object directly
      std::bind(&OrbSlam3Node::raw_img_callback,
                this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ORB_SLAM3 node started");
  }

private:
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
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (frame.empty())
    {
        RCLCPP_INFO(get_logger(), "Received empty frame");
        return;
    }
    double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    m_slam->TrackMonocular(frame, ts);
  }

  image_transport::Subscriber
    m_compressed_img_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
    m_raw_img_sub;
    std::string m_vocab_path;
    std::string m_setting_path;
    std::string m_image_topic;
    std::unique_ptr<ORB_SLAM3::System> m_slam;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrbSlam3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
