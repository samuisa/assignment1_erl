#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>

#include "assignment_msg/msg/aruco_info.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <string>

namespace assignment1_erl
{

    class ImageAnnotator : public rclcpp::Node
    {
    public:
        explicit ImageAnnotator(const rclcpp::NodeOptions &options)
            : Node("image_annotator", options)
        {
            // Output folder (must already exist)
            declare_parameter<std::string>("output_dir", "marker_images");
            output_dir_ = get_parameter("output_dir").as_string();

            // Camera image subscription
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                "/camera/image", rclcpp::SensorDataQoS(),
                std::bind(&ImageAnnotator::image_callback, this,
                          std::placeholders::_1));

            // Notification from the FSM (Finite State Machine)
            marker_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/marker_reached", 10,
                std::bind(&ImageAnnotator::marker_callback, this, std::placeholders::_1));

            // Annotated image publisher
            image_pub_ = image_transport::create_publisher(this, "/marker_image");

            RCLCPP_INFO(get_logger(), "ImageAnnotator started, saving to '%s'", output_dir_.c_str());
        }

    private:
        // Subscriptions and publisher
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr marker_sub_;
        image_transport::Publisher image_pub_;

        // Fixed output folder (must already exist)
        std::string output_dir_;

        // Store recent images and their timestamps
        struct Frame
        {
            rclcpp::Time stamp;
            cv::Mat image;
        };

        std::deque<Frame> frames_;

        // Last annotated image for each marker ID
        std::map<int, sensor_msgs::msg::Image::SharedPtr> outputs_;

        // Receive and store camera images
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
        {
            try
            {
                auto image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                Frame frame;
                frame.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
                frame.image = image->image.clone();

                frames_.push_back(frame);

                // Keep only the last 60 images
                if (frames_.size() > 60)
                    frames_.pop_front();
            }
            catch (const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(get_logger(), "%s", e.what());
            }
        }

        // Called when the FSM reports that a marker was reached
        void marker_callback(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (frames_.empty())
                return;

            // Find the image closest to the detection timestamp
            int best_index = -1;
            double min_difference = 1e9;

            rclcpp::Time target_stamp(msg->header.stamp, get_clock()->get_clock_type());

            for (int i = 0; i < static_cast<int>(frames_.size()); i++)
            {
                double difference =
                    std::abs((frames_[i].stamp - target_stamp).seconds());

                if (difference < min_difference)
                {
                    min_difference = difference;
                    best_index = i;
                }
            }

            // Check that the image is close enough in time
            if (min_difference > 0.1)
            {
                RCLCPP_WARN(get_logger(), "Image not found");
                return;
            }

            // Copy the selected image
            cv::Mat image = frames_[best_index].image.clone();

            // Marker center and radius
            cv::Point center(static_cast<int>(msg->u), static_cast<int>(msg->v));
            int radius = static_cast<int>(msg->s / 2.0);

            // Draw circle
            cv::circle(image, center, radius, cv::Scalar(0, 255, 0), 3);

            // Draw marker ID
            std::string text = "ID: " + std::to_string(msg->marker_id);

            cv::putText(image, text,
                        cv::Point(center.x, std::max(20, center.y - radius - 10)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2);

            // Save the annotated image to disk (PNG)
            std::string path = output_dir_ + "/marker_" +
                               std::to_string(msg->marker_id) + ".png";

            if (cv::imwrite(path, image))
                RCLCPP_INFO(get_logger(), "Saved %s", path.c_str());
            else
                RCLCPP_ERROR(get_logger(), "Failed to save %s", path.c_str());

            // Convert back to ROS image and publish
            auto output = cv_bridge::CvImage(
                              msg->header,
                              sensor_msgs::image_encodings::BGR8,
                              image)
                              .toImageMsg();

            outputs_[msg->marker_id] = output;
            image_pub_.publish(output);

            RCLCPP_INFO(get_logger(), "Published image for marker %d", msg->marker_id);
        }
    };

} // namespace assignment1_erl

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::ImageAnnotator)