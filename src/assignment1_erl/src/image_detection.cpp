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

            RCLCPP_INFO(get_logger(), "Detection node started (marker_size=%.3f m, desired_size=%.1f px)",
                        marker_size_, desired_size_);

            cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                    if (msg->k[0] > 0.0 && msg->k[4] > 0.0)
                    {
                        const bool first_calibration = fx_ <= 0.0 || fy_ <= 0.0;
                        fx_ = msg->k[0];
                        fy_ = msg->k[4];
                        cx_ = msg->k[2];
                        cy_ = msg->k[5];
                        if (first_calibration)
                            RCLCPP_INFO(get_logger(),
                                        "Camera calibration received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                                        fx_, fy_, cx_, cy_);
                    }
                    else
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                             "Ignoring invalid camera calibration (fx=%.2f, fy=%.2f)",
                                             msg->k[0], msg->k[4]);
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
            if (fx_ <= 0.0 || fy_ <= 0.0)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Waiting for valid camera calibration...");
                return;
            }

            for (const auto &m : msg->markers)
            {
                if (m.marker_id != target_id_) continue;
                if (!std::isfinite(m.pose.position.z) || m.pose.position.z <= 0.0)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "Ignoring target %d: invalid depth z=%.3f",
                                         m.marker_id, m.pose.position.z);
                    continue;
                }

                assignment_msg::msg::ArucoInfo info;
                info.header = msg->header;
                info.marker_id = m.marker_id;
                info.u = static_cast<float>(fx_ * (m.pose.position.x / m.pose.position.z) + cx_);
                info.v = static_cast<float>(fy_ * (m.pose.position.y / m.pose.position.z) + cy_);
                info.s = static_cast<float>(fx_ * marker_size_ / m.pose.position.z);
                info.image_center_x = static_cast<float>(cx_);
                info.desired_size = static_cast<float>(desired_size_);
                pub_->publish(info);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "Target %d detected: xyz=(%.2f, %.2f, %.2f) m -> uv=(%.1f, %.1f), size=%.1f px",
                                     info.marker_id, m.pose.position.x, m.pose.position.y,
                                     m.pose.position.z, info.u, info.v, info.s);
                break;
            }
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::DetectionManager)