#include <array>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>
#include "assignment_msg/msg/aruco_info.hpp"
#include "assignment_msg/msg/fsm_status.hpp"

namespace assignment1_erl
{
    class FsmManager : public rclcpp::Node
    {
    public:
        explicit FsmManager(const rclcpp::NodeOptions &options) : Node("fsm_node", options)
        {
            RCLCPP_INFO(get_logger(), "FSM started: state=IDLE (0), target=%d", target_);
            fsm_pub_ = create_publisher<assignment_msg::msg::FsmStatus>("/fsm_status", 10);
            reached_pub_ = create_publisher<assignment_msg::msg::ArucoInfo>("/marker_reached", 10);
            aruco_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/aruco_info", 10, [this](const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg) { aruco_cb(msg); });
            maneuver_sub_ = create_subscription<std_msgs::msg::Empty>(
                "/maneuver_complete", 10, [this](const std_msgs::msg::Empty::ConstSharedPtr) {
                    if (state_ != 1) return;
                    if (target_ >= 4)
                    {
                        state_ = 4;
                        RCLCPP_INFO(get_logger(),
                                    "Transition: ROTATE (1) -> FINISHED (4) | visited=[%d,%d,%d,%d]",
                                    visited_[1], visited_[2], visited_[3], visited_[4]);
                    }
                    else
                    {
                        ++target_;
                        state_ = 3;
                        RCLCPP_INFO(get_logger(),
                                    "Transition: ROTATE (1) -> SEARCH (3) | next_target=%d, visited=[%d,%d,%d,%d]",
                                    target_, visited_[1], visited_[2], visited_[3], visited_[4]);
                    }
                    publish_state();
                });
            timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
                if (state_ == 0)
                {
                    state_ = 3;
                    RCLCPP_INFO(get_logger(), "Transition: IDLE (0) -> SEARCH (3) | target=%d", target_);
                }
                publish_state();
            });
        }

    private:
        // Stati: 0=IDLE, 1=ROTATE, 2=APPROACH, 3=SEARCH, 4=FINISHED
        int state_{0}, target_{1};
        std::array<bool, 5> visited_{};

        rclcpp::Publisher<assignment_msg::msg::FsmStatus>::SharedPtr fsm_pub_;
        rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr reached_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr maneuver_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void publish_state()
        {
            assignment_msg::msg::FsmStatus s;
            s.current_state = state_;
            s.target_id = target_;
            fsm_pub_->publish(s);
        }

        void aruco_cb(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (target_ < 1 || target_ > 4 || msg->marker_id != target_ || visited_[target_]) return;

            if (state_ == 3)
            {
                state_ = 2;
                RCLCPP_INFO(get_logger(), "Transition: SEARCH (3) -> APPROACH (2) | target=%d", target_);
                publish_state();
            }
            else if (state_ == 2 &&
                     std::abs(msg->u - msg->image_center_x) <= 10.0 &&
                     msg->s >= msg->desired_size - 10.0F)
            {
                visited_[target_] = true;
                reached_pub_->publish(*msg);
                state_ = (target_ == 4) ? 4 : 1;
                RCLCPP_INFO(get_logger(),
                            "Target %d reached -> Transition: APPROACH (2) -> %s (%d) | visited=[%d,%d,%d,%d]",
                            target_, state_ == 4 ? "FINISHED" : "ROTATE", state_,
                            visited_[1], visited_[2], visited_[3], visited_[4]);
                publish_state();
            }
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::FsmManager)