#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "gazebo_ath/msg/aruco_info.hpp"
#include "gazebo_ath/msg/fsm_status.hpp"

namespace gazebo_ath
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
            fsm_pub_ = create_publisher<msg::FsmStatus>("/fsm_status", 10);
            aruco_info_sub_ = create_subscription<msg::ArucoInfo>(
                "/aruco_info", 10,
                std::bind(&FsmManager::aruco_callback, this, std::placeholders::_1));
            maneuver_complete_sub_ = create_subscription<std_msgs::msg::Empty>(
                "/maneuver_complete", 10,
                std::bind(&FsmManager::maneuver_complete_callback, this, std::placeholders::_1));
            timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&FsmManager::logic_loop, this));

            RCLCPP_INFO(get_logger(), "FSM pronta: avanzo alla ricerca del marker 1.");
        }

    private:
        static constexpr double kDesiredSizeTolerancePx = 5.0;
        static constexpr double kCenterTolerancePx = 5.0;

        State_t current_state_{STATE_IDLE};
        int expected_marker_{1};

        rclcpp::Publisher<msg::FsmStatus>::SharedPtr fsm_pub_;
        rclcpp::Subscription<msg::ArucoInfo>::SharedPtr aruco_info_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr maneuver_complete_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void publish_state()
        {
            msg::FsmStatus status;
            status.current_state = current_state_;
            status.target_id = expected_marker_;
            fsm_pub_->publish(status);
        }

        void aruco_callback(const msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (msg->marker_id != expected_marker_)
            {
                return;
            }

            if (current_state_ == STATE_SEARCH)
            {
                current_state_ = STATE_APPROACH;
                RCLCPP_INFO(get_logger(), "Marker %d rilevato: inizio avvicinamento.", expected_marker_);
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

            if (expected_marker_ == 4)
            {
                current_state_ = STATE_FINISHED;
                RCLCPP_INFO(get_logger(), "Marker 4 raggiunto: tesoro trovato.");
            }
            else
            {
                current_state_ = STATE_ROTATE;
                RCLCPP_INFO(get_logger(), "Marker %d raggiunto: eseguo l'istruzione.", expected_marker_);
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
            current_state_ = STATE_SEARCH;
            RCLCPP_INFO(get_logger(), "Rotazione completata: avanzo alla ricerca del marker %d.",
                        expected_marker_);
            publish_state();
        }

        void logic_loop()
        {
            if (current_state_ == STATE_IDLE)
            {
                current_state_ = STATE_SEARCH;
            }
            publish_state();
        }
    };
} // namespace gazebo_ath

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_ath::FsmManager)
