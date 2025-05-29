#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class VideoPubNode: public rclcpp::Node
{
public:
    VideoPubNode() : Node("video_publisher")
    {
        this->declare_parameter<std::string>("video_path","");
        this->declare_parameter<std::string>("camera_name", "video_cam");
        this->declare_parameter<int>("queue_size", 10);
        this->get_parameter("video_path", m_video_path);
        this->get_parameter("queue_size", m_queue_size);
        this->get_parameter("camera_name", m_cam_name);

        m_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", m_queue_size);

        m_cap.open(m_video_path);
        if(!m_cap.isOpened())
        {
            RCLCPP_ERROR(get_logger(), "Failed to open video: %s", m_video_path.c_str());
            rclcpp::shutdown();
            return;
        }

        double fps = m_cap.get(cv::CAP_PROP_FPS);
        if(fps <= 0)
            fps = 30.0;
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(int(1000.0/fps)),
            std::bind(&VideoPubNode::publish_image, this));

        RCLCPP_INFO(get_logger(), "VideoPublisher ready, publishing %s", m_video_path.c_str());
    }

private:
    void publish_image()
    {
        cv::Mat frame;
        m_cap >> frame;
        if (frame.empty())
        {
            RCLCPP_INFO(get_logger(), "End of video stream");
            rclcpp::shutdown();
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = m_cam_name;

        auto cv_img = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        m_image_pub->publish(*cv_img);
        // RCLCPP_INFO(get_logger(), "Published image");
    }

    std::string m_video_path;
    std::string m_cam_name;
    int m_queue_size{10};
    cv::VideoCapture m_cap;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}