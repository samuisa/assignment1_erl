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
    /**
     * @brief Converts FSM and marker data into smooth velocity commands for the robot.
     *
     * The node searches for markers, aligns with the active marker, approaches it,
     * and performs relative odometry-based turns between markers.
     */
    class NavigationManager : public rclcpp::Node
    {
    public:
        /**
         * @brief Creates the navigation node, its publishers and subscriptions.
         * @param options ROS 2 node options supplied by the component container.
         */
        explicit NavigationManager(const rclcpp::NodeOptions &options)
            : Node("navigation_node", options),
              last_marker_update_(0, 0, get_clock()->get_clock_type())
        {
            cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            maneuver_complete_pub_ =
                create_publisher<std_msgs::msg::Empty>("/maneuver_complete", 10);

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                std::bind(&NavigationManager::odom_callback, this, std::placeholders::_1));
            aruco_info_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/aruco_info", 10,
                std::bind(&NavigationManager::aruco_info_callback, this, std::placeholders::_1));
            fsm_status_sub_ = create_subscription<assignment_msg::msg::FsmStatus>(
                "/fsm_status", 10,
                std::bind(&NavigationManager::fsm_status_callback, this, std::placeholders::_1));

            timer_ = create_wall_timer(
                std::chrono::milliseconds(25),
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

        static constexpr float kLinearGain = 0.0014F;      ///< Proportional gain for distance correction.
        static constexpr float kAlignmentGain = 0.0022F;   ///< Proportional gain for image alignment.
        static constexpr float kYawGain = 1.4F;            ///< Proportional gain for heading correction.
        static constexpr float searchingSpeed = 0.18F;     ///< Forward speed while searching, in m/s.
        static constexpr float kMaxLinearSpeed = 0.25F;    ///< Maximum linear speed, in m/s.
        static constexpr float kMaxAngularSpeed = 0.75F;   ///< Maximum angular speed, in rad/s.
        static constexpr float kFarAngularSpeed = 0.35F;   ///< Angular speed cap for distant markers.
        static constexpr float kFarMarkerSizePx = 25.0F;   ///< Threshold below which marker is far (>3 m).
        static constexpr float kSmoothingAlpha = 0.20F;    ///< Low-pass filter factor at 40 Hz.
        static constexpr float kRefDesiredSizePx = 166.0F; ///< Reference desired size in pixels.
        static constexpr float kRefFocalLengthPx = 381.4F; ///< Reference camera focal length in pixels.

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr maneuver_complete_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_info_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<assignment_msg::msg::FsmStatus>::SharedPtr fsm_status_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        float s_ = 0.0F;
        float u_ = 0.0F;
        float cx_ = 320.0F;
        float desired_size_ = 166.0F;
        float actual_yaw_ = 0.0F;
        float last_yaw_ = 0.0F;
        float rotated_angle_ = 0.0F;
        float target_rotation_ = 0.0F;
        float estimated_target_yaw_ = 0.0F;
        float last_linear_cmd_ = 0.0F;
        float last_angular_cmd_ = 0.0F;
        float last_seen_dir_ = 1.0F;
        int current_state_ = 0;
        int target_id_ = 1;
        bool have_marker_data_ = false;
        bool have_odometry_ = false;
        bool have_estimated_yaw_ = false;
        bool rotation_initialized_ = false;
        bool rotation_complete_sent_ = false;
        rclcpp::Time last_marker_update_;

        /**
         * @brief Updates the robot's current yaw from odometry.
         * @param msg Odometry message containing the robot orientation quaternion.
         */
        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
            const auto &q = msg->pose.pose.orientation;
            actual_yaw_ = std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            have_odometry_ = true;
        }

        /**
         * @brief Updates the active FSM state and resets state-specific data as needed.
         * @param msg FSM state and target marker ID.
         */
        void fsm_status_callback(const assignment_msg::msg::FsmStatus::ConstSharedPtr msg)
        {
            const int previous_state = current_state_;
            const int previous_target = target_id_;
            current_state_ = msg->current_state;
            target_id_ = msg->target_id;

            if (target_id_ != previous_target)
            {
                have_marker_data_ = false;
                have_estimated_yaw_ = false;
                last_seen_dir_ = 1.0F;
            }

            if (current_state_ == rotating && previous_state != rotating)
            {
                rotation_initialized_ = false;
                rotation_complete_sent_ = false;
            }
        }

        /**
         * @brief Stores measurements and updates the odometry-referenced marker bearing.
         * @param msg Marker ID, image position, and apparent marker size.
         */
        void aruco_info_callback(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (msg->marker_id != target_id_)
            {
                return;
            }

            s_ = msg->s;
            u_ = msg->u;
            cx_ = msg->image_center_x;
            desired_size_ = msg->desired_size;
            last_seen_dir_ = (cx_ - u_ >= 0.0F) ? 1.0F : -1.0F;
            have_marker_data_ = true;
            last_marker_update_ = this->now();

            if (have_odometry_)
            {
                const float focal_px = (desired_size_ > 0.0F)
                                           ? (desired_size_ * kRefFocalLengthPx / kRefDesiredSizePx)
                                           : kRefFocalLengthPx;
                const float bearing_offset = std::atan2(cx_ - u_, focal_px);
                estimated_target_yaw_ = std::atan2(
                    std::sin(actual_yaw_ + bearing_offset),
                    std::cos(actual_yaw_ + bearing_offset));
                have_estimated_yaw_ = true;
            }
        }

        /**
         * @brief Returns the route turn angle associated with the active target.
         * @return Turn offset in radians.
         */
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

        /**
         * @brief Computes and publishes smoothed velocity commands.
         */
        void control_loop()
        {
            geometry_msgs::msg::Twist target_vel;

            switch (current_state_)
            {
            case searching:
                target_vel.linear.x = searchingSpeed;
                break;

            case rotating:
            {
                if (!have_odometry_)
                {
                    break;
                }

                if (!rotation_initialized_)
                {
                    last_yaw_ = actual_yaw_;
                    rotated_angle_ = 0.0F;
                    target_rotation_ = rotation_offset();
                    rotation_initialized_ = true;
                }

                const float delta_yaw = std::atan2(
                    std::sin(actual_yaw_ - last_yaw_),
                    std::cos(actual_yaw_ - last_yaw_));
                rotated_angle_ += delta_yaw;
                last_yaw_ = actual_yaw_;

                const float yaw_error = target_rotation_ - rotated_angle_;

                if (std::abs(yaw_error) > 0.05F)
                {
                    target_vel.angular.z = std::clamp(
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
            {
                if (!have_marker_data_)
                {
                    break;
                }

                const double time_since_seen = (this->now() - last_marker_update_).seconds();

                if (time_since_seen <= 0.50)
                {
                    const float u_error = cx_ - u_;
                    const float size_error = desired_size_ - s_;
                    const float angular_limit =
                        (s_ < kFarMarkerSizePx) ? kFarAngularSpeed : kMaxAngularSpeed;

                    if (std::abs(u_error) > 0.5F)
                    {
                        target_vel.angular.z = std::clamp(
                            kAlignmentGain * u_error,
                            -angular_limit, angular_limit);
                    }

                    if (std::abs(size_error) > 1.0F)
                    {
                        const float alignment_scale =
                            std::clamp(1.0F - std::abs(u_error) / 300.0F, 0.15F, 1.0F);
                        target_vel.linear.x = std::clamp(
                            kLinearGain * size_error * alignment_scale,
                            -kMaxLinearSpeed, kMaxLinearSpeed);
                    }
                }
                else if (time_since_seen <= 4.0 && have_odometry_ && have_estimated_yaw_)
                {
                    const float yaw_error = std::atan2(
                        std::sin(estimated_target_yaw_ - actual_yaw_),
                        std::cos(estimated_target_yaw_ - actual_yaw_));

                    target_vel.angular.z = std::clamp(
                        1.2F * yaw_error, -kFarAngularSpeed, kFarAngularSpeed);

                    if (std::abs(yaw_error) < 0.25F)
                    {
                        target_vel.linear.x = 0.18F;
                    }
                }
                else
                {
                    target_vel.angular.z = last_seen_dir_ * 0.25F;
                }
                break;
            }

            case 0:
            case finished:
            default:
                break;
            }

            last_linear_cmd_ +=
                kSmoothingAlpha * (static_cast<float>(target_vel.linear.x) - last_linear_cmd_);
            last_angular_cmd_ +=
                kSmoothingAlpha * (static_cast<float>(target_vel.angular.z) - last_angular_cmd_);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = last_linear_cmd_;
            cmd.angular.z = last_angular_cmd_;
            cmd_vel_pub_->publish(cmd);
        }
    };
} // namespace assignment1_erl

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::NavigationManager)