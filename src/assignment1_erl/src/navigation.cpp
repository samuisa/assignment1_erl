#include <algorithm>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>

#include "assignment1_erl/msg/aruco_info.hpp"
#include "assignment1_erl/msg/fsm_status.hpp"

namespace assignment1_erl
{
    class NavigationManager : public rclcpp::Node
    {
    public:
        explicit NavigationManager(const rclcpp::NodeOptions &options)
            : Node("navigation_node", options)
        {
            cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            maneuver_complete_pub_ =
                create_publisher<std_msgs::msg::Empty>("/maneuver_complete", 10);

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                std::bind(&NavigationManager::odom_callback, this, std::placeholders::_1));
            aruco_info_sub_ = create_subscription<msg::ArucoInfo>(
                "/aruco_info", 10,
                std::bind(&NavigationManager::aruco_info_callback, this, std::placeholders::_1));
            fsm_status_sub_ = create_subscription<msg::FsmStatus>(
                "/fsm_status", 10,
                std::bind(&NavigationManager::fsm_status_callback, this, std::placeholders::_1));

            timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&NavigationManager::control_loop, this));

        }

        ~NavigationManager()
        {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        }

    private:
        static constexpr int rotating = 1;
        static constexpr int approaching = 2;
        static constexpr int searching = 3;
        static constexpr int finished = 4;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr maneuver_complete_pub_;
        rclcpp::Subscription<msg::ArucoInfo>::SharedPtr aruco_info_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<msg::FsmStatus>::SharedPtr fsm_status_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        const float kLinearGain = 0.001F;
        const float kAlignmentGain = 0.003F;
        const float kYawGain = 1.5F;
        const float searchingSpeed = 0.1F;
        const float kMaxLinearSpeed = 0.2F;
        const float kMaxAngularSpeed = 0.8F;
        const float kPixelTolerance = 5.0F;

        float s_ = 0.0F;
        float u_ = 0.0F;
        float cx_ = 320.0F;
        float desired_size_ = 166.0F;
        float target_yaw_ = 0.0F;
        float actual_yaw_ = 0.0F;
        int current_state_ = 0;
        int target_id_ = 1;
        bool have_marker_data_ = false;
        bool have_odometry_ = false;
        bool rotation_initialized_ = false;
        bool rotation_complete_sent_ = false;
        std::chrono::steady_clock::time_point last_marker_update_;

        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
            const auto &q = msg->pose.pose.orientation;
            actual_yaw_ = std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            have_odometry_ = true;
        }

        void fsm_status_callback(const msg::FsmStatus::ConstSharedPtr msg)
        {
            const int previous_state = current_state_;
            const int previous_target = target_id_;
            current_state_ = msg->current_state;
            target_id_ = msg->target_id;

            if (target_id_ != previous_target)
            {
                have_marker_data_ = false;
            }

            if (current_state_ == rotating && previous_state != rotating)
            {
                rotation_initialized_ = false;
                rotation_complete_sent_ = false;
            }
        }

        void aruco_info_callback(const msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (msg->marker_id != target_id_)
            {
                return;
            }

            s_ = msg->s;
            u_ = msg->u;
            cx_ = msg->image_center_x;
            desired_size_ = msg->desired_size;
            have_marker_data_ = true;
            last_marker_update_ = std::chrono::steady_clock::now();
        }

        float rotation_offset() const
        {
            switch (target_id_)
            {
            case 1:
                return static_cast<float>(-M_PI / 2.0);
            case 2:
                return static_cast<float>(M_PI);
            case 3:
                return static_cast<float>(M_PI / 2.0);
            default:
                return 0.0F;
            }
        }

        void control_loop()
        {
            geometry_msgs::msg::Twist velocity;

            switch (current_state_)
            {
            case searching:
                velocity.linear.x = searchingSpeed;
                break;

            case rotating:
            {
                if (!have_odometry_)
                {
                    break;
                }

                if (!rotation_initialized_)
                {
                    const float offset = rotation_offset();
                    target_yaw_ = std::atan2(
                        std::sin(actual_yaw_ + offset),
                        std::cos(actual_yaw_ + offset));
                    rotation_initialized_ = true;
                }

                const float yaw_error = std::atan2(
                    std::sin(target_yaw_ - actual_yaw_),
                    std::cos(target_yaw_ - actual_yaw_));

                if (std::abs(yaw_error) > 0.05F)
                {
                    velocity.angular.z = std::clamp(
                        kYawGain * yaw_error, -kMaxAngularSpeed, kMaxAngularSpeed);
                }
                else if (!rotation_complete_sent_)
                {
                    maneuver_complete_pub_->publish(std_msgs::msg::Empty());
                    rotation_complete_sent_ = true;
                }
                break;
            }

            case approaching:
                if (!have_marker_data_ ||
                    std::chrono::steady_clock::now() - last_marker_update_ >
                        std::chrono::milliseconds(500))
                {
                    break;
                }

                if (std::abs(u_ - cx_) > kPixelTolerance)
                {
                    velocity.angular.z = std::clamp(
                        kAlignmentGain * (cx_ - u_),
                        -kMaxAngularSpeed, kMaxAngularSpeed);
                }
                else if (std::abs(desired_size_ - s_) > kPixelTolerance)
                {
                    velocity.linear.x = std::clamp(
                        kLinearGain * (desired_size_ - s_),
                        -kMaxLinearSpeed, kMaxLinearSpeed);
                }
                break;

            case 0:
            case finished:
            default:
                break;
            }

            cmd_vel_pub_->publish(velocity);
        }
    };
} // namespace assignment1_erl

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::NavigationManager)
