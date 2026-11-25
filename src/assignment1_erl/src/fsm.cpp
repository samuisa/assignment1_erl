/**
 * @file fsm.cpp
 * @brief Finite State Machine (FSM) node that coordinates the treasure hunt.
 *
 * The node decides what the robot has to do at every moment. It receives the
 * marker measurements from the detection node (/aruco_info) and the
 * "maneuver finished" notification from the navigation node (/maneuver_complete),
 * and publishes the current state and the current target on /fsm_status.
 *
 * States:
 *  - 0 IDLE      : node just started, nothing received yet
 *  - 1 ROTATE    : marker reached, the robot executes the rotation associated to it
 *  - 2 APPROACH  : target marker seen, the robot moves towards it
 *  - 3 SEARCH    : target marker not seen yet, the robot looks for it
 *  - 4 FINISHED  : last marker (4) reached, the hunt is over
 */
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>
#include "assignment_msg/msg/aruco_info.hpp"
#include "assignment_msg/msg/fsm_status.hpp"

namespace assignment1_erl
{
    /**
     * @class FsmManager
     * @brief Robot Operating System 2 (ROS 2) node implementing the Finite State Machine (FSM) of the treasure hunt.
     *
     * Subscribes to:
     *  - /aruco_info        (ArucoInfo): position and size of the target marker in the image
     *  - /maneuver_complete (Empty)    : sent by the navigation node when the rotation is done
     *
     * Publishes:
     *  - /fsm_status        (FsmStatus): current state and current target marker
     *  - /marker_reached    (ArucoInfo): measurement of a marker at the moment it is considered reached
     */
    class FsmManager : public rclcpp::Node
    {
    public:
        /**
         * @brief Creates publishers, subscribers and the 50 ms timer that drives the state machine.
         * @param options Node options (needed to load the node as a component).
         */
        explicit FsmManager(const rclcpp::NodeOptions &options) : Node("fsm_node", options)
        {
            fsm_pub_ = create_publisher<assignment_msg::msg::FsmStatus>("/fsm_status", 10);
            reached_pub_ = create_publisher<assignment_msg::msg::ArucoInfo>("/marker_reached", 10);
            aruco_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/aruco_info", 10, [this](const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg) { aruco_cb(msg); });
            maneuver_sub_ = create_subscription<std_msgs::msg::Empty>(
                "/maneuver_complete", 10, [this](const std_msgs::msg::Empty::ConstSharedPtr) { maneuver_cb(); });

            last_info_ = now();
            timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { timer_cb(); });

            RCLCPP_INFO(get_logger(), "State machine started: state IDLE, initial target = marker %d", target_);
            RCLCPP_INFO(get_logger(), "Publishing on: %s, %s", fsm_pub_->get_topic_name(), reached_pub_->get_topic_name());
            RCLCPP_INFO(get_logger(), "Listening on: %s, %s", aruco_sub_->get_topic_name(), maneuver_sub_->get_topic_name());
        }

    private:
        // Stati: 0=IDLE, 1=ROTATE, 2=APPROACH, 3=SEARCH, 4=FINISHED
        int state_{0}, target_{1};      ///< Current state and current target marker identifier (ID)
        std::array<bool, 5> visited_{}; ///< visited_[i] is true once marker i has been reached (index 0 unused)
        rclcpp::Time last_info_;        ///< Time of the last /aruco_info message received

        rclcpp::Publisher<assignment_msg::msg::FsmStatus>::SharedPtr fsm_pub_;
        rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr reached_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr maneuver_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        /**
         * @brief Converts a state number into a readable name (used only for logging).
         * @param s State number (0..4).
         * @return Name of the state, or "?" if the number is unknown.
         */
        static const char *state_name(int s)
        {
            switch (s)
            {
            case 0: return "IDLE";
            case 1: return "ROTATE";
            case 2: return "APPROACH";
            case 3: return "SEARCH";
            case 4: return "FINISHED";
            default: return "?";
            }
        }

        /**
         * @brief Builds a comma-separated string with the markers already reached (e.g. "1,2").
         * @return The list of reached markers, or "none" if no marker has been reached yet.
         */
        std::string visited_str() const
        {
            std::string out;
            for (int i = 1; i < 5; ++i)
                if (visited_[i]) out += (out.empty() ? "" : ",") + std::to_string(i);
            return out.empty() ? "none" : out;
        }

        /**
         * @brief Changes the state and prints the transition to the terminal.
         * @param new_state State to switch to.
         * @param reason    Short text explaining why the transition happens (logged).
         * @note Publishing the new state is left to the caller.
         */
        // Cambia stato e lo scrive a terminale (la pubblicazione resta a carico del chiamante)
        void set_state(int new_state, const char *reason)
        {
            RCLCPP_INFO(get_logger(), "STATE: %s -> %s | reason: %s | target=%d",
                        state_name(state_), state_name(new_state), reason, target_);
            state_ = new_state;
        }

        /**
         * @brief Publishes the current state and target on /fsm_status.
         */
        void publish_state()
        {
            assignment_msg::msg::FsmStatus s;
            s.current_state = state_;
            s.target_id = target_;
            fsm_pub_->publish(s);
        }

        /**
         * @brief Periodic callback (every 50 ms).
         *
         * Leaves IDLE and starts the search at the first call, republishes the
         * state continuously (so that the other nodes always know it), and prints
         * warnings/status logs if no marker information arrives while approaching.
         */
        void timer_cb()
        {
            if (state_ == 0) set_state(3, "startup: starting the search");
            publish_state();

            const double silence = (now() - last_info_).seconds();

            // In APPROACH i messaggi devono arrivare di continuo: se spariscono il marker e' stato perso
            if (state_ == 2 && silence > 1.0)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "APPROACH: no /aruco_info for %.1f s (marker %d lost or out of view)", silence, target_);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "State=%s | target=%d | completed markers=[%s] | last /aruco_info %.1f s ago",
                                 state_name(state_), target_, visited_str().c_str(), silence);
        }

        /**
         * @brief Callback of /maneuver_complete: the rotation after a marker is finished.
         *
         * Only accepted in ROTATE. It moves the target to the next marker and
         * goes back to SEARCH.
         */
        void maneuver_cb()
        {
            if (state_ != 1)
            {
                RCLCPP_WARN(get_logger(), "/maneuver_complete received in state %s: ignored", state_name(state_));
                return;
            }
            RCLCPP_INFO(get_logger(), "Maneuver completed for marker %d: moving on to marker %d", target_, target_ + 1);
            ++target_;
            set_state(3, "maneuver completed, searching for the next marker");
            publish_state();
        }

        /**
         * @brief Callback of /aruco_info: measurement of the target marker in the image.
         * @param msg Marker data (identifier, pixel position u/v, size s, image center, desired size).
         *
         * - In SEARCH: the first valid measurement of the target switches to APPROACH.
         * - In APPROACH: if both the horizontal error (u) and the size error (s) are within
         *   10 pixels, the marker is considered reached: it is published on /marker_reached
         *   and the state becomes ROTATE (or FINISHED if it was marker 4).
         * Measurements of markers different from the target, or already visited, are ignored.
         */
        void aruco_cb(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            last_info_ = now();

            if (msg->marker_id != target_ || visited_[target_]) return;

            const float u_err = msg->u - msg->image_center_x;
            const float s_err = msg->desired_size - msg->s;

            if (state_ == 3)
            {
                RCLCPP_INFO(get_logger(), "Marker %d seen in SEARCH: u_err=%+.1f px, s=%.1f px, desired=%.1f px",
                            target_, u_err, msg->s, msg->desired_size);
                set_state(2, "target marker seen");
                publish_state();
            }
            else if (state_ == 2)
            {
                const bool u_ok = std::abs(u_err) <= 10.0F;
                const bool s_ok = std::abs(s_err) <= 10.0F;

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "APPROACH marker %d: u_err=%+.1f px [%s], s_err=%+.1f px [%s] (s=%.1f, desired=%.1f, tolerance 10 px)",
                                     target_, u_err, u_ok ? "ok" : "out", s_err, s_ok ? "ok" : "out", msg->s, msg->desired_size);

                if (u_ok && s_ok)
                {
                    visited_[target_] = true;
                    reached_pub_->publish(*msg);

                    if (target_ == 4)
                    {
                        RCLCPP_INFO(get_logger(), "*** TREASURE FOUND! Marker 4 reached: the treasure hunt is over. ***");
                        set_state(4, "marker 4 reached");
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "Marker %d REACHED (u_err=%+.1f px, s_err=%+.1f px): published on /marker_reached, starting the maneuver",
                                    target_, u_err, s_err);
                        set_state(1, "marker reached, executing the instruction");
                    }
                    publish_state();
                }
            }
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::FsmManager)
