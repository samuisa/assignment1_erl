#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_components/register_node_macro.hpp> // Required for ROS 2 components

namespace gazebo_ath
{

    class NavigationManager : public rclcpp::Node
    {
    public:
        // Components require a constructor that accepts NodeOptions
        explicit NavigationManager(const rclcpp::NodeOptions &options)
            : Node("navigation_node", options)
        {
            // Publisher per muovere il robot su /cmd_vel
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            // fsm_status = this->create_subscription<>("", 10);

            // Timer per pubblicare costantemente la velocità
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&NavigationManager::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Nodo di navigazione lineare avviato. Il robot andrà solo dritto.");
        }

        ~NavigationManager()
        {
            // Ferma il robot prima di chiudere il nodo
            auto stop_vel = geometry_msgs::msg::Twist();
            stop_vel.linear.x = 0.0;
            stop_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_vel);
        }

    private:
        k_v = 0.2; // Velocità lineare costante in avanti
        k_w = 0.2;

        void timer_callback()
        {
            auto vel = geometry_msgs::msg::Twist();
            vel.linear.x = 0.2;  // Imposta la velocità lineare a 0.2 m/s in avanti
            vel.angular.z = 0.0; // Assicurati che non ci sia rotazione

            cmd_vel_pub_->publish(vel);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void fsm_status_callback(const std::shared_ptr<YourFsmStatusMessageType> msg)
        {

            switch (msg->state)
            {
            case YourFsmStatusMessageType::State::STOP:
                // Ferma il robot
                auto stop_vel = geometry_msgs::msg::Twist();
                stop_vel.linear.x = 0.0;
                stop_vel.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_vel);
                break;

            case YourFsmStatusMessageType::State::ROTATE:
                auto rotate_vel = geometry_msgs::msg::Twist();
                rotate_vel.linear.x = 0.0;
                if (marker.id == 1)
                {
                    target_yaw = 90.0; // Imposta l'angolo target a 90 gradi
                    while (abs(target_yaw - actual_yaw) > 0.1)
                    {
                        rotate_vel.angular.z = k_w * (target_yaw - actual_yaw);
                        cmd_vel_pub_->publish(rotate_vel);
                    }
                    YourFsmStatusMessageType::State::MOVE;
                }
                if (marker.id == 2)
                {
                    target_yaw = 180.0; // Imposta l'angolo target a 180 gradi
                    while (abs(target_yaw - actual_yaw) > 0.1)
                    {
                        rotate_vel.angular.z = k_w * (target_yaw - actual_yaw);
                        cmd_vel_pub_->publish(rotate_vel);
                    }
                    YourFsmStatusMessageType::State::MOVE;
                }
                if (marker.id == 3)
                {
                    target_yaw = -90.0; // Imposta l'angolo target a 270 gradi
                    while (abs(target_yaw - actual_yaw) > 0.1)
                    {
                        rotate_vel.angular.z = k_w * (target_yaw - actual_yaw);
                        cmd_vel_pub_->publish(rotate_vel);
                    }
                    YourFsmStatusMessageType::State::MOVE;
                }
                rotate_vel.angular.z = 1.0; // Imposta una velocità angolare
                cmd_vel_pub_->publish(rotate_vel);
                break;

            case YourFsmStatusMessageType::State::MOVE:
                auto forward_vel = geometry_msgs::msg::Twist();

                while (abs(sd - s) > 166)
                {

                    if (abs(u - cx) < 0.1)
                    {
                        forward_vel.linear.x = k_v * (sd - s);
                        forward_vel.angular.z = 0.0;
                        cmd_vel_pub_->publish(forward_vel);
                    }
                    else
                    {
                        forward_vel.linear.x = 0.0;
                        forward_vel.angular.z = k_w * (u - cx);
                        cmd_vel_pub_->publish(forward_vel);
                    }
                }
                if (marker.id != 4)
                {
                    YourFsmStatusMessageType::State::ROTATE;
                }
                else
                {
                    YourFsmStatusMessageType::State::STOP;
                }

                break;

            default:
                // Gestisci altri stati se necessario
                break;
            }
        }
    };

} // namespace gazebo_ath

// Register the component so it can be dynamically loaded by the container
RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_ath::NavigationManager)