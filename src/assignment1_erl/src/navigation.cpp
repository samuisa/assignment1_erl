#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include "assignment_msg/msg/aruco_info.hpp"
#include "assignment_msg/msg/fsm_status.hpp"

namespace assignment1_erl
{
    class NavigationManager : public rclcpp::Node
    {
    public:
        explicit NavigationManager(const rclcpp::NodeOptions &options)
            : Node("navigation_node", options), last_seen_(0, 0, get_clock()->get_clock_type())
        {
            RCLCPP_INFO(get_logger(), "Navigation node started");
            cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            done_pub_ = create_publisher<std_msgs::msg::Empty>("/maneuver_complete", 10);

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                const auto &q = msg->pose.pose.orientation;
                yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                has_odom_ = true;
            });

            aruco_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>("/aruco_info", 10, [this](const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg) {
                if (msg->marker_id != target_) return;
                s_ = msg->s; u_ = msg->u; cx_ = msg->image_center_x; desired_ = msg->desired_size;
                has_marker_ = true;
                last_seen_ = now();
            });

            fsm_sub_ = create_subscription<assignment_msg::msg::FsmStatus>("/fsm_status", 10, [this](const assignment_msg::msg::FsmStatus::ConstSharedPtr msg) {
                if (msg->target_id != target_) { has_marker_ = false; }
                if (msg->current_state == 1 && state_ != 1) { rot_init_ = false; rot_sent_ = false; }
                state_ = msg->current_state;
                target_ = msg->target_id;
            });

            timer_ = create_wall_timer(std::chrono::milliseconds(25), [this]() { control_loop(); });
        }

        ~NavigationManager() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<assignment_msg::msg::FsmStatus>::SharedPtr fsm_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        float s_ = 0.0F, u_ = 0.0F, cx_ = 320.0F, desired_ = 120.0F;
        float yaw_ = 0.0F, last_yaw_ = 0.0F, rot_acc_ = 0.0F;
        float lin_cmd_ = 0.0F, ang_cmd_ = 0.0F;
        int state_ = 0, target_ = 1;
        bool has_marker_ = false, has_odom_ = false, rot_init_ = false, rot_sent_ = false;
        rclcpp::Time last_seen_;

        void control_loop()
        {
            float target_lin = 0.0F, target_ang = 0.0F;
            float rotation_goal = 0.0F, rotation_error = 0.0F;
            float u_error = 0.0F, size_error = 0.0F;
            const double marker_age = has_marker_ ? (now() - last_seen_).seconds() : -1.0;

            if (state_ == 3) // SEARCH
            {
                target_lin = 0.18F;
            }
            else if (state_ == 1 && has_odom_) // ROTATE
            {
                if (!rot_init_) { last_yaw_ = yaw_; rot_acc_ = 0.0F; rot_init_ = true; }
                rot_acc_ += std::atan2(std::sin(yaw_ - last_yaw_), std::cos(yaw_ - last_yaw_));
                last_yaw_ = yaw_;

                rotation_goal = static_cast<float>((target_ == 1) ? -M_PI / 2.0 : (target_ == 2) ? M_PI : (target_ == 3) ? M_PI / 2.0 : 0.0);
                rotation_error = rotation_goal - rot_acc_;

                if (std::abs(rotation_error) > 0.05F && !rot_sent_)
                {
                    target_ang = std::clamp(1.4F * rotation_error, -0.75F, 0.75F);
                }
                else
                {
                    target_ang = 0.0F;
                    ang_cmd_ = 0.0F;
                    if (!rot_sent_)
                    {
                        RCLCPP_INFO(get_logger(), "Rotation completed for target %d (acc=%.2f/%.2f rad)",
                                    target_, rot_acc_, rotation_goal);
                        done_pub_->publish(std_msgs::msg::Empty());
                        rot_sent_ = true;
                    }
                }
            }
            else if (state_ == 2 && has_marker_) // APPROACH
            {
                if (marker_age <= 0.5)
                {
                    u_error = cx_ - u_;
                    size_error = desired_ - s_;

                    if (std::abs(u_error) > 8.0F)
                        target_ang = std::clamp(0.012F * u_error, -0.40F, 0.40F);

                    if (size_error > 10.0F)
                        target_lin = std::clamp(0.004F * size_error, 0.05F, 0.15F);
                }
                else
                {
                    target_lin = 0.05F;
                    target_ang = 0.0F;
                }
            }

            lin_cmd_ += 0.20F * (target_lin - lin_cmd_);
            ang_cmd_ += 0.20F * (target_ang - ang_cmd_);

            if (state_ == 3)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "[SEARCH] target=%d | cmd=(lin=%.2f, ang=%.2f)",
                                     target_, lin_cmd_, ang_cmd_);
            }
            else if (state_ == 2)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "[APPROACH] target=%d | u_err=%.1f px, size_err=%.1f px, age=%.2fs | cmd=(lin=%.2f, ang=%.2f)",
                                     target_, u_error, size_error, marker_age, lin_cmd_, ang_cmd_);
            }
            else if (state_ == 1)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "[ROTATE] target=%d | odom=%s, rot=%.2f/%.2f rad (err=%.2f) | cmd=(lin=%.2f, ang=%.2f)",
                                     target_, has_odom_ ? "ok" : "missing", rot_acc_, rotation_goal,
                                     rotation_error, lin_cmd_, ang_cmd_);
            }

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = lin_cmd_;
            cmd.angular.z = ang_cmd_;
            cmd_pub_->publish(cmd);
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::NavigationManager)