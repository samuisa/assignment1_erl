#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <stdexcept>
#include "assignment_msg/msg/aruco_info.hpp"
#include "assignment_msg/msg/fsm_status.hpp"

namespace assignment1_erl
{
    class DetectionManager : public rclcpp::Node
    {
    public:
        explicit DetectionManager(const rclcpp::NodeOptions &options) : Node("detection_node", options)
        {
            marker_size_ = declare_parameter<double>("marker_size", 0.2);
            desired_size_ = declare_parameter<double>("desired_size_px", 120.0);
            if (!std::isfinite(marker_size_) || marker_size_ <= 0.0)
                throw std::invalid_argument("marker_size must be a positive finite value");
            if (!std::isfinite(desired_size_) || desired_size_ <= 0.0)
                throw std::invalid_argument("desired_size_px must be a positive finite value");

            cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                    if (msg->k[0] > 0.0 && msg->k[4] > 0.0)
                    {
                        fx_ = msg->k[0];
                        fy_ = msg->k[4];
                        cx_ = msg->k[2];
                        cy_ = msg->k[5];
                    }
                });

            fsm_sub_ = create_subscription<assignment_msg::msg::FsmStatus>(
                "/fsm_status", 10,
                [this](const assignment_msg::msg::FsmStatus::ConstSharedPtr msg) {
                    target_id_ = msg->target_id;
                });

            aruco_sub_ = create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections", 10,
                [this](const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg) { aruco_cb(msg); });

            pub_ = create_publisher<assignment_msg::msg::ArucoInfo>("/aruco_info", 10);
        }

    private:
        double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
        double marker_size_, desired_size_;
        int target_id_ = 1;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
        rclcpp::Subscription<assignment_msg::msg::FsmStatus>::SharedPtr fsm_sub_;
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
        rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr pub_;

        void aruco_cb(const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg)
        {
            if (fx_ <= 0.0 || fy_ <= 0.0 || marker_size_ <= 0.0 || desired_size_ <= 0.0) return;

            for (const auto &m : msg->markers)
            {
                // Ignora completamente qualsiasi marker diverso dal target attuale
                if (m.marker_id != target_id_ || !std::isfinite(m.pose.position.z) ||
                    m.pose.position.z <= 0.0) continue;

                assignment_msg::msg::ArucoInfo info;
                info.header = msg->header;
                info.marker_id = m.marker_id;
                info.u = static_cast<float>(fx_ * (m.pose.position.x / m.pose.position.z) + cx_);
                info.v = static_cast<float>(fy_ * (m.pose.position.y / m.pose.position.z) + cy_);
                info.s = static_cast<float>(fx_ * marker_size_ / m.pose.position.z);
                info.image_center_x = static_cast<float>(cx_);
                info.desired_size = static_cast<float>(desired_size_);
                pub_->publish(info);
                break; // Trovato il target, esce subito dal ciclo
            }
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::DetectionManager)