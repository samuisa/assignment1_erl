#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <cmath>
#include <stdexcept>
#include "gazebo_ath/msg/aruco_info.hpp"

namespace gazebo_ath
{
    class DetectionManager : public rclcpp::Node
    {
    public:
        explicit DetectionManager(const rclcpp::NodeOptions &options)
            : Node("detection_node", options)
        {
            marker_size_m_ = declare_parameter<double>("marker_size_m", 0.2);
            desired_size_sim_px_ = declare_parameter<double>("desired_size_sim_px", 166.0);
            simulation_focal_length_px_ =
                declare_parameter<double>("simulation_focal_length_px", 381.4);

            if (!std::isfinite(marker_size_m_) || marker_size_m_ <= 0.0 ||
                !std::isfinite(desired_size_sim_px_) || desired_size_sim_px_ <= 0.0 ||
                !std::isfinite(simulation_focal_length_px_) || simulation_focal_length_px_ <= 0.0)
            {
                throw std::invalid_argument("Marker size and camera focal lengths must be positive.");
            }

            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", 10,
                std::bind(&DetectionManager::camera_info_callback, this, std::placeholders::_1));

            aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections", 10,
                std::bind(&DetectionManager::aruco_callback, this, std::placeholders::_1));

            aruco_info_pub_ = this->create_publisher<gazebo_ath::msg::ArucoInfo>("/aruco_info", 10);

        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
        rclcpp::Publisher<gazebo_ath::msg::ArucoInfo>::SharedPtr aruco_info_pub_;

        double f_ = 0.0;
        double cx_ = 0.0;
        double marker_size_m_{};
        double desired_size_sim_px_{};
        double simulation_focal_length_px_{};
        bool camera_info_received_ = false;

        void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
        {
            if (std::isfinite(msg->k[0]) && msg->k[0] > 0.0 &&
                std::isfinite(msg->k[2]))
            {
                f_ = msg->k[0];
                cx_ = msg->k[2];
                camera_info_received_ = true;
            }
        }

        void aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg)
        {
            if (!camera_info_received_ || msg->markers.empty())
                return;

            gazebo_ath::msg::ArucoInfo info_msg;

            for (const auto &marker : msg->markers)
            {
                float aruco_x = marker.pose.position.x;
                float aruco_z = marker.pose.position.z;

                if (aruco_z > 0.0)
                {
                    info_msg.marker_id = marker.marker_id;
                    info_msg.u = static_cast<float>(f_ * (aruco_x / aruco_z) + cx_);
                    info_msg.s = static_cast<float>(f_ * marker_size_m_ / aruco_z);
                    info_msg.image_center_x = static_cast<float>(cx_);
                    info_msg.desired_size =
                        static_cast<float>(desired_size_sim_px_ * f_ / simulation_focal_length_px_);

                    aruco_info_pub_->publish(info_msg);
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Marker %d detected: u=%.1f px, size=%.1f px (target size=%.1f px).",
                        static_cast<int>(info_msg.marker_id), info_msg.u, info_msg.s,
                        info_msg.desired_size);
                }
            }
        }
    };
} // namespace gazebo_ath

RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_ath::DetectionManager)