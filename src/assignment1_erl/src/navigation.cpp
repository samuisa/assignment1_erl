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

            if (state_ == 3) // SEARCH
            {
                target_lin = 0.18F;
            }
            else if (state_ == 1 && has_odom_) // ROTATE
            {
                if (!rot_init_) { last_yaw_ = yaw_; rot_acc_ = 0.0F; rot_init_ = true; }
                rot_acc_ += std::atan2(std::sin(yaw_ - last_yaw_), std::cos(yaw_ - last_yaw_));
                last_yaw_ = yaw_;

                float goal = (target_ == 1) ? -M_PI / 2.0 : (target_ == 2) ? M_PI : (target_ == 3) ? M_PI / 2.0 : 0.0;
                float err = static_cast<float>(goal) - rot_acc_;

                if (std::abs(err) > 0.05F && !rot_sent_)
                {
                    target_ang = std::clamp(1.4F * err, -0.75F, 0.75F);
                }
                else
                {
                    target_ang = 0.0F;
                    ang_cmd_ = 0.0F; // Azzera il filtro per fermare subito la rotazione
                    if (!rot_sent_)
                    {
                        done_pub_->publish(std_msgs::msg::Empty());
                        rot_sent_ = true;
                    }
                }
            }
            else if (state_ == 2 && has_marker_) // APPROACH
            {
                double dt = (now() - last_seen_).seconds();
                if (dt <= 0.5)
                {
                    float u_err = cx_ - u_;
                    float s_err = desired_ - s_;

                    if (std::abs(u_err) > 8.0F)
                        target_ang = std::clamp(0.012F * u_err, -0.40F, 0.40F);

                    if (s_err > 10.0F) // Avanza SOLO se è ancora più piccolo del target
                        target_lin = std::clamp(0.004F * s_err, 0.05F, 0.15F); // Rallenta man mano che si avvicina
                }
                else
                {
                    target_lin = 0.0F; // Se perde il marker durante l'approach, si ferma invece di schiantarsi
                    target_ang = 0.0F;
                }
            }

            lin_cmd_ += 0.20F * (target_lin - lin_cmd_);
            ang_cmd_ += 0.20F * (target_ang - ang_cmd_);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = lin_cmd_;
            cmd.angular.z = ang_cmd_;
            cmd_pub_->publish(cmd);
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::NavigationManager)