#include <array>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "assignment_msg/msg/aruco_info.hpp"
#include "assignment_msg/msg/fsm_status.hpp"

namespace assignment1_erl
{
    enum State_t
    {
        STATE_IDLE = 0,
        STATE_ROTATE = 1,
        STATE_APPROACH = 2,
        STATE_SEARCH = 3,
        STATE_FINISHED = 4,
    };

    class FsmManager : public rclcpp::Node
    {
    public:
        explicit FsmManager(const rclcpp::NodeOptions &options)
            : Node("fsm_node", options)
        {
            fsm_pub_ = create_publisher<assignment_msg::msg::FsmStatus>("/fsm_status", 10);
            aruco_info_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/aruco_info", 10,
                std::bind(&FsmManager::aruco_callback, this, std::placeholders::_1));
            maneuver_complete_sub_ = create_subscription<std_msgs::msg::Empty>(
                "/maneuver_complete", 10,
                std::bind(&FsmManager::maneuver_complete_callback, this, std::placeholders::_1));
            timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&FsmManager::logic_loop, this));

        }

    private:
        static constexpr double kDesiredSizeTolerancePx = 5.0;
        static constexpr double kCenterTolerancePx = 5.0;
        static constexpr int kFirstMarkerId = 1;
        static constexpr int kLastMarkerId = 4;
        static constexpr auto kMarkerLostTimeout = std::chrono::milliseconds(1000);

        State_t current_state_{STATE_IDLE};
        int expected_marker_{1};
        std::array<bool, kLastMarkerId + 1> visited_markers_{};
        std::chrono::steady_clock::time_point last_expected_marker_update_;

        rclcpp::Publisher<assignment_msg::msg::FsmStatus>::SharedPtr fsm_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_info_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr maneuver_complete_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        static const char *state_name(State_t state)
        {
            switch (state)
            {
            case STATE_IDLE:
                return "idle";
            case STATE_ROTATE:
                return "rotate";
            case STATE_APPROACH:
                return "approach";
            case STATE_SEARCH:
                return "search";
            case STATE_FINISHED:
                return "finished";
            default:
                return "unknown";
            }
        }

        void transition_to(State_t next_state, const char *reason)
        {
            if (current_state_ == next_state)
            {
                return;
            }

            const State_t previous_state = current_state_;
            current_state_ = next_state;
            RCLCPP_INFO(
                get_logger(), "FSM state: %s -> %s (target marker %d): %s.",
                state_name(previous_state), state_name(current_state_), expected_marker_, reason);
        }

        void publish_state()
        {
            assignment_msg::msg::FsmStatus status;
            status.current_state = current_state_;
            status.target_id = expected_marker_;
            fsm_pub_->publish(status);
        }

        void aruco_callback(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (msg->marker_id < kFirstMarkerId || msg->marker_id > kLastMarkerId)
            {
                return;
            }

            if (visited_markers_[msg->marker_id])
            {
                RCLCPP_DEBUG_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Ignoring marker %d: it has already been visited.", msg->marker_id);
                return;
            }

            if (msg->marker_id != expected_marker_)
            {
                return;
            }

            last_expected_marker_update_ = std::chrono::steady_clock::now();

            if (current_state_ == STATE_SEARCH)
            {
                transition_to(STATE_APPROACH, "expected marker detected; starting approach");
                publish_state();
                return;
            }

            if (current_state_ != STATE_APPROACH)
            {
                return;
            }

            const bool aligned = std::abs(msg->u - msg->image_center_x) <= kCenterTolerancePx;
            const bool at_desired_distance =
                std::abs(msg->desired_size - msg->s) <= kDesiredSizeTolerancePx;
            if (!aligned || !at_desired_distance)
            {
                return;
            }

            visited_markers_[expected_marker_] = true;
            RCLCPP_INFO(get_logger(), "Marker %d reached and marked as visited.", expected_marker_);

            if (expected_marker_ == 4)
            {
                transition_to(STATE_FINISHED, "final marker reached");
            }
            else
            {
                transition_to(STATE_ROTATE, "marker reached; starting maneuver");
            }
            publish_state();
        }

        void maneuver_complete_callback(const std_msgs::msg::Empty::ConstSharedPtr)
        {
            if (current_state_ != STATE_ROTATE)
            {
                return;
            }

            ++expected_marker_;
            transition_to(STATE_SEARCH, "maneuver complete; resuming search");
            publish_state();
        }

        void logic_loop()
        {
            if (current_state_ == STATE_IDLE)
            {
                transition_to(STATE_SEARCH, "starting search");
            }
            else if (current_state_ == STATE_APPROACH &&
                     std::chrono::steady_clock::now() - last_expected_marker_update_ >
                         kMarkerLostTimeout)
            {
                RCLCPP_WARN(
                    get_logger(), "Lost marker %d during approach; resuming search.",
                    expected_marker_);
                transition_to(STATE_SEARCH, "expected marker is no longer visible");
            }
            publish_state();
        }
    };
} // namespace assignment1_erl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::FsmManager)
