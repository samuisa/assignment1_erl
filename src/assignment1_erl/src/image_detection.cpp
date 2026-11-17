
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include <cmath>
#include <stdexcept>
#include <functional>

#include "assignment_msg/msg/aruco_info.hpp"

namespace assignment1_erl
{

class DetectionManager : public rclcpp::Node
{
public:

    DetectionManager()
        : Node("detection_node")
    {
        marker_size_m_ =
            declare_parameter<double>("marker_size_m", 0.2);

        desired_size_sim_px_ =
            declare_parameter<double>("desired_size_sim_px", 166.0);

        simulation_focal_length_px_ =
            declare_parameter<double>("simulation_focal_length_px", 381.4);

        if (!std::isfinite(marker_size_m_) ||
            marker_size_m_ <= 0.0 ||
            !std::isfinite(desired_size_sim_px_) ||
            desired_size_sim_px_ <= 0.0 ||
            !std::isfinite(simulation_focal_length_px_) ||
            simulation_focal_length_px_ <= 0.0)
        {
            throw std::invalid_argument(
                "Marker size and camera focal lengths must be positive.");
        }

        camera_info_sub_ =
            create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", 10,
                std::bind(&DetectionManager::camera_info_callback,
                          this, std::placeholders::_1));

        aruco_sub_ =
            create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections", 10,
                std::bind(&DetectionManager::aruco_callback,
                          this, std::placeholders::_1));

        aruco_info_pub_ =
            create_publisher<assignment_msg::msg::ArucoInfo>(
                "/aruco_info", 10);

        RCLCPP_INFO(get_logger(), "Image detection node started.");
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;

    rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_info_pub_;

    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;

    double marker_size_m_{};
    double desired_size_sim_px_{};
    double simulation_focal_length_px_{};

    bool camera_info_received_ = false;


    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
        if (std::isfinite(msg->k[0]) &&
            msg->k[0] > 0.0 &&
            std::isfinite(msg->k[4]) &&
            msg->k[4] > 0.0 &&
            std::isfinite(msg->k[2]) &&
            std::isfinite(msg->k[5]))
        {
            fx_ = msg->k[0];
            fy_ = msg->k[4];
            cx_ = msg->k[2];
            cy_ = msg->k[5];

            camera_info_received_ = true;
        }
    }


    void aruco_callback(
        const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg)
    {
        if (!camera_info_received_ || msg->markers.empty()) {
            return;
        }

        for (const auto &marker : msg->markers)
        {
            const float aruco_x = marker.pose.position.x;
            const float aruco_y = marker.pose.position.y;
            const float aruco_z = marker.pose.position.z;

            if (aruco_z <= 0.0) {
                continue;
            }

            assignment_msg::msg::ArucoInfo info_msg;

            // Preserve the original image timestamp and frame.
            info_msg.header = msg->header;

            info_msg.marker_id = marker.marker_id;

            // Horizontal center of the marker in pixels.
            info_msg.u = static_cast<float>(
                fx_ * (aruco_x / aruco_z) + cx_);

            // Vertical center of the marker in pixels.
            info_msg.v = static_cast<float>(
                fy_ * (aruco_y / aruco_z) + cy_);

            // Marker size in pixels.
            info_msg.s = static_cast<float>(
                fx_ * marker_size_m_ / aruco_z);

            info_msg.image_center_x =
                static_cast<float>(cx_);

            info_msg.desired_size = static_cast<float>(
                desired_size_sim_px_ *
                fx_ / simulation_focal_length_px_);

            aruco_info_pub_->publish(info_msg);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Marker %d detected: u=%.1f, v=%.1f, size=%.1f px (target=%.1f px).",
                static_cast<int>(info_msg.marker_id),
                info_msg.u,
                info_msg.v,
                info_msg.s,
                info_msg.desired_size);
        }
    }
};

} // namespace assignment1_erl


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<assignment1_erl::DetectionManager>());

    rclcpp::shutdown();

    return 0;
}
