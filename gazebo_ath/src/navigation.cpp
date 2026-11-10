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
        void timer_callback()
        {
            auto vel = geometry_msgs::msg::Twist();
            vel.linear.x = 0.2;  // Imposta la velocità lineare a 0.2 m/s in avanti
            vel.angular.z = 0.0; // Assicurati che non ci sia rotazione

            cmd_vel_pub_->publish(vel);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

} // namespace gazebo_ath

// Register the component so it can be dynamically loaded by the container
RCLCPP_COMPONENTS_REGISTER_NODE(gazebo_ath::NavigationManager)