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
    class FsmManager : public rclcpp::Node
    {
    public:
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

            RCLCPP_INFO(get_logger(), "Macchina a stati avviata: stato IDLE, target iniziale = marker %d", target_);
            RCLCPP_INFO(get_logger(), "Pubblico su: %s, %s", fsm_pub_->get_topic_name(), reached_pub_->get_topic_name());
            RCLCPP_INFO(get_logger(), "Ascolto su: %s, %s", aruco_sub_->get_topic_name(), maneuver_sub_->get_topic_name());
        }

    private:
        // Stati: 0=IDLE, 1=ROTATE, 2=APPROACH, 3=SEARCH, 4=FINISHED
        int state_{0}, target_{1};
        std::array<bool, 5> visited_{};
        rclcpp::Time last_info_;

        rclcpp::Publisher<assignment_msg::msg::FsmStatus>::SharedPtr fsm_pub_;
        rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr reached_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr maneuver_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

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

        std::string visited_str() const
        {
            std::string out;
            for (int i = 1; i < 5; ++i)
                if (visited_[i]) out += (out.empty() ? "" : ",") + std::to_string(i);
            return out.empty() ? "nessuno" : out;
        }

        // Cambia stato e lo scrive a terminale (la pubblicazione resta a carico del chiamante)
        void set_state(int new_state, const char *reason)
        {
            RCLCPP_INFO(get_logger(), "STATO: %s -> %s | motivo: %s | target=%d",
                        state_name(state_), state_name(new_state), reason, target_);
            state_ = new_state;
        }

        void publish_state()
        {
            assignment_msg::msg::FsmStatus s;
            s.current_state = state_;
            s.target_id = target_;
            fsm_pub_->publish(s);
        }

        void timer_cb()
        {
            if (state_ == 0) set_state(3, "avvio: parto con la ricerca");
            publish_state();

            const double silence = (now() - last_info_).seconds();

            // In APPROACH i messaggi devono arrivare di continuo: se spariscono il marker e' stato perso
            if (state_ == 2 && silence > 1.0)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "APPROACH: nessun /aruco_info da %.1f s (marker %d perso o fuori campo)", silence, target_);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Stato=%s | target=%d | marker completati=[%s] | ultimo /aruco_info %.1f s fa",
                                 state_name(state_), target_, visited_str().c_str(), silence);
        }

        void maneuver_cb()
        {
            if (state_ != 1)
            {
                RCLCPP_WARN(get_logger(), "/maneuver_complete ricevuto in stato %s: ignorato", state_name(state_));
                return;
            }
            RCLCPP_INFO(get_logger(), "Manovra completata per il marker %d: passo al marker %d", target_, target_ + 1);
            ++target_;
            set_state(3, "manovra completata, cerco il marker successivo");
            publish_state();
        }

        void aruco_cb(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            last_info_ = now();

            if (msg->marker_id != target_ || visited_[target_]) return;

            const float u_err = msg->u - msg->image_center_x;
            const float s_err = msg->desired_size - msg->s;

            if (state_ == 3)
            {
                RCLCPP_INFO(get_logger(), "Marker %d visto in SEARCH: u_err=%+.1f px, s=%.1f px, desired=%.1f px",
                            target_, u_err, msg->s, msg->desired_size);
                set_state(2, "marker target visto");
                publish_state();
            }
            else if (state_ == 2)
            {
                const bool u_ok = std::abs(u_err) <= 10.0F;
                const bool s_ok = std::abs(s_err) <= 10.0F;

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "APPROACH marker %d: u_err=%+.1f px [%s], s_err=%+.1f px [%s] (s=%.1f, desired=%.1f, tolleranza 10 px)",
                                     target_, u_err, u_ok ? "ok" : "fuori", s_err, s_ok ? "ok" : "fuori", msg->s, msg->desired_size);

                if (u_ok && s_ok)
                {
                    visited_[target_] = true;
                    reached_pub_->publish(*msg);

                    if (target_ == 4)
                    {
                        RCLCPP_INFO(get_logger(), "*** TESORO TROVATO! Marker 4 raggiunto: la caccia al tesoro e' conclusa. ***");
                        set_state(4, "marker 4 raggiunto");
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "Marker %d RAGGIUNTO (u_err=%+.1f px, s_err=%+.1f px): pubblicato su /marker_reached, avvio la manovra",
                                    target_, u_err, s_err);
                        set_state(1, "marker raggiunto, eseguo l'istruzione");
                    }
                    publish_state();
                }
            }
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::FsmManager)
