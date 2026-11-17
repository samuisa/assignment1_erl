
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>

#include "assignment_msg/msg/aruco_info.hpp"

#include <deque>
#include <functional>
#include <cmath>

namespace assignment1_erl
{

class ImageAnnotator : public rclcpp::Node
{
public:

    ImageAnnotator() : Node("image_annotator")
    {
        // Camera image subscription
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageAnnotator::image_callback, this,
                      std::placeholders::_1));

        // Notification from FSM
        marker_sub_ =
            create_subscription<assignment_msg::msg::ArucoInfo>(
                "/marker_reached", 10,
                std::bind(&ImageAnnotator::marker_callback, this,
                          std::placeholders::_1));

        // Annotated image publisher
        image_pub_ = image_transport::create_publisher(
            this, "/marker_image");

        RCLCPP_INFO(get_logger(), "ImageAnnotator started");
    }

private:

    // Subscriptions and publisher
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Subscription<
        assignment_msg::msg::ArucoInfo>::SharedPtr marker_sub_;

    image_transport::Publisher image_pub_;


    // Store recent images and their timestamps
    struct Frame
    {
        rclcpp::Time stamp;
        cv::Mat image;
    };

    std::deque<Frame> frames_;


    // Receive and store camera images
    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try
        {
            auto image = cv_bridge::toCvCopy(
                msg, sensor_msgs::image_encodings::BGR8);

            Frame frame;
            frame.stamp = rclcpp::Time(
                msg->header.stamp, get_clock()->get_clock_type());

            frame.image = image->image.clone();

            frames_.push_back(frame);

            // Keep only the last 60 images
            if (frames_.size() > 60)
                frames_.pop_front();
        }
        catch (const cv_bridge::Exception & e)
        {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
    }


    // Called when FSM reports that a marker was reached
    void marker_callback(
        const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
    {
        if (frames_.empty())
            return;

        // Find the image closest to the detection timestamp
        int best_index = -1;
        double min_difference = 1e9;

        rclcpp::Time target_stamp(
            msg->header.stamp, get_clock()->get_clock_type());

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
        cv::Point center(msg->u, msg->v);
        int radius = static_cast<int>(msg->s / 2.0);

        // Draw circle
        cv::circle(image, center, radius,
                   cv::Scalar(0, 255, 0), 3);

        // Draw marker ID
        std::string text = "ID: " + std::to_string(msg->marker_id);

        cv::putText(image, text,
                    cv::Point(center.x, center.y - radius - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2);

        // Convert back to ROS image and publish
        auto output = cv_bridge::CvImage(
            msg->header,
            sensor_msgs::image_encodings::BGR8,
            image).toImageMsg();

        image_pub_.publish(output);

        RCLCPP_INFO(get_logger(), "Published image for marker %d",
                    msg->marker_id);
    }
};

} // namespace assignment1_erl


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<assignment1_erl::ImageAnnotator>());

    rclcpp::shutdown();

    return 0;
}
