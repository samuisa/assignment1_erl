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
                if (!has_odom_)
                    RCLCPP_INFO(get_logger(), "Prima odometria ricevuta: yaw=%.1f deg", yaw_ * kRad2Deg);
                has_odom_ = true;
            });

            aruco_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>("/aruco_info", 10, [this](const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg) {
                if (msg->marker_id != target_) return;
                s_ = msg->s; u_ = msg->u; cx_ = msg->image_center_x; desired_ = msg->desired_size;
                if (!has_marker_)
                    RCLCPP_INFO(get_logger(), "Marker %d visto per la prima volta: u=%.1f px, s=%.1f px, desired=%.1f px",
                                target_, u_, s_, desired_);
                has_marker_ = true;
                last_seen_ = now();
            });

            fsm_sub_ = create_subscription<assignment_msg::msg::FsmStatus>("/fsm_status", 10, [this](const assignment_msg::msg::FsmStatus::ConstSharedPtr msg) {
                const int new_state = static_cast<int>(msg->current_state);
                const int new_target = static_cast<int>(msg->target_id);

                if (new_target != target_)
                {
                    RCLCPP_INFO(get_logger(), "Target cambiato: marker %d -> marker %d (azzero la memoria del marker visto)", target_, new_target);
                    has_marker_ = false;
                }
                if (new_state != state_)
                    RCLCPP_INFO(get_logger(), "Stato: %s -> %s (target %d)", state_name(state_), state_name(new_state), new_target);

                if (new_state == 1 && state_ != 1) { rot_init_ = false; rot_sent_ = false; }
                state_ = new_state;
                target_ = new_target;
            });

            timer_ = create_wall_timer(std::chrono::milliseconds(25), [this]() { control_loop(); });

            RCLCPP_INFO(get_logger(), "Navigazione avviata. Pubblico su: %s, %s", cmd_pub_->get_topic_name(), done_pub_->get_topic_name());
            RCLCPP_INFO(get_logger(), "Ascolto su: %s, %s, %s", odom_sub_->get_topic_name(), aruco_sub_->get_topic_name(), fsm_sub_->get_topic_name());
        }

        ~NavigationManager() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

    private:
        static constexpr float kRad2Deg = 57.2957795F;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<assignment_msg::msg::FsmStatus>::SharedPtr fsm_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        float s_ = 0.0F, u_ = 0.0F, cx_ = 320.0F, desired_ = 140.0F;
        float yaw_ = 0.0F, last_yaw_ = 0.0F, rot_acc_ = 0.0F;
        float lin_cmd_ = 0.0F, ang_cmd_ = 0.0F;
        int state_ = 0, target_ = 1;
        bool has_marker_ = false, has_odom_ = false, rot_init_ = false, rot_sent_ = false;
        rclcpp::Time last_seen_;

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

        void control_loop()
        {
            float target_lin = 0.0F, target_ang = 0.0F;

            if (state_ == 0) // IDLE: nessun /fsm_status ricevuto
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "IDLE: nessun /fsm_status ricevuto (macchina a stati spenta? namespace o dominio diversi?)");
            }
            else if (state_ == 3) // SEARCH
            {
                target_lin = 0.08F;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "SEARCH: cerco il marker %d, avanzo dritto (target lin=%.2f m/s)", target_, target_lin);
            }
            else if (state_ == 1) // ROTATE
            {
                if (!has_odom_)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "ROTATE: nessuna odometria ricevuta, il robot NON ruota (controlla il topic dell'odometria)");
                }
                else
                {
                    float goal = (target_ == 1) ? -M_PI / 2.0 : (target_ == 2) ? M_PI : (target_ == 3) ? M_PI / 2.0 : 0.0;

                    if (!rot_init_)
                    {
                        last_yaw_ = yaw_; rot_acc_ = 0.0F; rot_init_ = true;
                        RCLCPP_INFO(get_logger(), "ROTATE: inizio la rotazione per il marker %d, goal=%.1f deg (yaw iniziale=%.1f deg)",
                                    target_, goal * kRad2Deg, yaw_ * kRad2Deg);
                    }
                    rot_acc_ += std::atan2(std::sin(yaw_ - last_yaw_), std::cos(yaw_ - last_yaw_));
                    last_yaw_ = yaw_;

                    float err = goal - rot_acc_;

                    if (std::abs(err) > 0.05F && !rot_sent_)
                    {
                        target_ang = std::clamp(1.4F * err, -0.75F, 0.75F);
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                             "ROTATE: ruotato=%.1f deg, goal=%.1f deg, errore=%.1f deg -> ang=%.2f rad/s",
                                             rot_acc_ * kRad2Deg, goal * kRad2Deg, err * kRad2Deg, target_ang);
                    }
                    else
                    {
                        target_ang = 0.0F;
                        ang_cmd_ = 0.0F; // Azzera il filtro per fermare subito la rotazione
                        if (!rot_sent_)
                        {
                            RCLCPP_INFO(get_logger(), "ROTATE: completata (ruotato=%.1f deg, goal=%.1f deg, errore=%.2f deg): pubblico /maneuver_complete",
                                        rot_acc_ * kRad2Deg, goal * kRad2Deg, err * kRad2Deg);
                            done_pub_->publish(std_msgs::msg::Empty());
                            rot_sent_ = true;
                        }
                        else
                        {
                            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                                 "ROTATE: /maneuver_complete gia' inviato, attendo che la macchina a stati cambi stato");
                        }
                    }
                }
            }
            else if (state_ == 2) // APPROACH
            {
                if (!has_marker_)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "APPROACH: nessun /aruco_info ricevuto per il marker %d, robot fermo", target_);
                }
                else
                {
                    double dt = (now() - last_seen_).seconds();
                    if (dt <= 0.5)
                    {
                        float u_err = cx_ - u_;
                        float s_err = desired_ - s_;
                        const bool turn = std::abs(u_err) > 5.0F;
                        const bool move = std::abs(s_err) > 10.0F;

                        // Correzione angolare dolce per evitare pendolamenti
                        if (turn)
                            target_ang = std::clamp(0.015F * u_err, -0.45F, 0.45F);

                        // Avanza finche' |desired_ - s_| non rientra in 10 pixel (banda morta)
                        if (move)
                            target_lin = std::clamp(0.05F * s_err, -0.22F, 0.22F);

                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                             "APPROACH marker %d: u_err=%+.1f px, s_err=%+.1f px (s=%.1f, desired=%.1f) -> lin=%.2f m/s, ang=%.2f rad/s%s",
                                             target_, u_err, s_err, s_, desired_, target_lin, target_ang,
                                             (turn || move) ? "" : " [in tolleranza: attendo la conferma della macchina a stati]");
                    }
                    else
                    {
                        // Se perde il marker durante l'approccio, continua dritto senza fermarsi ne' oscillare
                        target_lin = 0.08F;
                        target_ang = 0.0F;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                             "APPROACH: marker %d perso da %.1f s, proseguo dritto a %.2f m/s", target_, dt, target_lin);
                    }
                }
            }
            else if (state_ == 4) // FINISHED
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "FINISHED: caccia al tesoro conclusa, robot fermo");
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
