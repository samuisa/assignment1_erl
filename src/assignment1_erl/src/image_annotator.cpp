#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include "assignment_msg/msg/aruco_info.hpp"

namespace assignment1_erl
{
    class ImageAnnotator : public rclcpp::Node
    {
    public:
        explicit ImageAnnotator(const rclcpp::NodeOptions &options) : Node("image_annotator", options)
        {
            out_dir_ = declare_parameter<std::string>("output_dir", "marker_images");
            img_pub_ = image_transport::create_publisher(this, "/marker_image");

            img_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                "camera/rgb/image_raw/compressed", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
                    try {
                        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                        frames_.push_back({rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()), cv_ptr->image});
                        if (frames_.size() > 30) frames_.pop_front();
                    } catch (...) {}
                });

            marker_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/marker_reached", 10, [this](const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg) { marker_cb(msg); });
        }

    private:
        std::string out_dir_;
        std::deque<std::pair<rclcpp::Time, cv::Mat>> frames_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr marker_sub_;
        image_transport::Publisher img_pub_;

        void marker_cb(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            if (frames_.empty()) return;

            rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
            auto best = std::min_element(frames_.begin(), frames_.end(), [&](const auto &a, const auto &b) {
                return std::abs((a.first - stamp).seconds()) < std::abs((b.first - stamp).seconds());
            });

            if (std::abs((best->first - stamp).seconds()) > 0.1) return;

            cv::Mat img = best->second.clone();
            cv::Point c(static_cast<int>(msg->u), static_cast<int>(msg->v));
            int r = static_cast<int>(msg->s / 2.0F);

            cv::circle(img, c, r, cv::Scalar(0, 255, 0), 3);
            cv::putText(img, "ID: " + std::to_string(msg->marker_id),
                        cv::Point(c.x, std::max(20, c.y - r - 10)), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            cv::imwrite(out_dir_ + "/marker_" + std::to_string(msg->marker_id) + ".png", img);
            img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, img).toImageMsg());
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::ImageAnnotator)