/**
 * @file image_annotator.cpp
 * @brief Node that saves and publishes an annotated picture of every marker reached.
 *
 * It keeps a small buffer with the latest camera frames. When the state machine
 * announces that a marker has been reached (/marker_reached), it picks the frame
 * closest in time to the measurement, draws a circle and the marker identifier on it,
 * saves it as a Portable Network Graphics (PNG) file and publishes it on /marker_image.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <string>
#include <system_error>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include "assignment_msg/msg/aruco_info.hpp"

namespace assignment1_erl
{
    /**
     * @class ImageAnnotator
     * @brief Robot Operating System 2 (ROS 2) node that annotates and stores the image of each reached marker.
     *
     * Parameters:
     *  - output_dir       (string): directory where the PNG files are saved (default "marker_images")
     *  - image_topic      (string): base image topic, without "/compressed" (default "camera/image")
     *  - image_compressed (bool)  : true to subscribe to the compressed topic, false for the raw one
     *
     * Subscribes to: camera images and /marker_reached.
     * Publishes: /marker_image (annotated image).
     */
    class ImageAnnotator : public rclcpp::Node
    {
    public:
        /**
         * @brief Reads the parameters, creates the output directory, the publisher,
         *        the image/marker subscribers and a watchdog that warns if frames stop arriving.
         * @param options Node options (needed to load the node as a component).
         */
        explicit ImageAnnotator(const rclcpp::NodeOptions &options) : Node("image_annotator", options)
        {
            out_dir_ = declare_parameter<std::string>("output_dir", "marker_images");
          
            const std::string image_topic = declare_parameter<std::string>("image_topic", "camera/image");
            
            const bool compressed = declare_parameter<bool>("image_compressed", true);

            
            std::error_code ec;
            std::filesystem::create_directories(out_dir_, ec);
            if (ec)
                RCLCPP_WARN(get_logger(), "Unable to create directory %s: %s", out_dir_.c_str(), ec.message().c_str());

            img_pub_ = image_transport::create_publisher(this, "/marker_image");

            if (compressed)
            {
                comp_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
                    image_topic + "/compressed", rclcpp::SensorDataQoS(),
                    [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { store_frame(msg); });
                in_topic_ = comp_sub_->get_topic_name();
            }
            else
            {
                raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
                    image_topic, rclcpp::SensorDataQoS(),
                    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { store_frame(msg); });
                in_topic_ = raw_sub_->get_topic_name();
            }

            marker_sub_ = create_subscription<assignment_msg::msg::ArucoInfo>(
                "/marker_reached", 10, [this](const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg) { marker_cb(msg); });

            
            last_frame_ = now();
            watchdog_ = create_wall_timer(std::chrono::seconds(1), [this]() {
                const double age = (now() - last_frame_).seconds();
                if (age > 3.0)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "No frame on %s for %.1f s: check image_topic, image_compressed, namespace and domain",
                                         in_topic_.c_str(), age);
            });

            RCLCPP_INFO(get_logger(), "Annotator started | images: %s (%s) | output directory: %s",
                        in_topic_.c_str(), compressed ? "compressed" : "raw", out_dir_.c_str());
            RCLCPP_INFO(get_logger(), "Listening for reached markers on %s", marker_sub_->get_topic_name());
        }

    private:
        std::string out_dir_, in_topic_;                          ///< Output directory and name of the image topic in use
        std::deque<std::pair<rclcpp::Time, cv::Mat>> frames_;     ///< Buffer of (timestamp, image) pairs, latest 30 frames
        rclcpp::Time last_frame_;                                 ///< Time of the last frame received (used by the watchdog)
        bool got_frame_ = false;                                  ///< True after the first frame has been received
        rclcpp::TimerBase::SharedPtr watchdog_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr comp_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr marker_sub_;
        image_transport::Publisher img_pub_;

        /**
         * @brief Converts an incoming image to OpenCV (BGR, 8 bit) and stores it in the buffer.
         * @tparam ImgPtr Pointer type of the message (raw Image or CompressedImage).
         * @param msg The received image message.
         *
         * The buffer keeps only the latest 30 frames. Frames that cannot be decoded are
         * skipped with a throttled warning.
         */
        // Salva il frame nel buffer (ultimi 30): funziona sia con Image sia con CompressedImage.
        template <typename ImgPtr>
        void store_frame(const ImgPtr &msg)
        {
            try
            {
                auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                frames_.push_back({rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()), cv_ptr->image});
                if (frames_.size() > 30) frames_.pop_front();

                last_frame_ = now();
                if (!got_frame_)
                {
                    got_frame_ = true;
                    RCLCPP_INFO(get_logger(), "First frame received on %s: %dx%d", in_topic_.c_str(), cv_ptr->image.cols, cv_ptr->image.rows);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Frame could not be decoded: %s", e.what());
            }
        }

        /**
         * @brief Callback of /marker_reached: creates, saves and publishes the annotated image.
         * @param msg Measurement of the marker at the moment it was reached.
         *
         * Steps: find the buffered frame closest to the message timestamp (it must be within
         * 0.1 s), draw a green circle centered on the marker with the marker identifier above it,
         * save it as <output_dir>/marker_<id>.png and publish it on /marker_image.
         */
        void marker_cb(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Marker %d reached (u=%.0f v=%.0f s=%.0f): looking for the closest frame, buffer=%zu frames",
                        msg->marker_id, msg->u, msg->v, msg->s, frames_.size());

            if (frames_.empty())
            {
                RCLCPP_WARN(get_logger(), "Marker %d reached but no frame received: check image_topic and image_compressed", msg->marker_id);
                return;
            }

            rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
            auto best = std::min_element(frames_.begin(), frames_.end(), [&](const auto &a, const auto &b) {
                return std::abs((a.first - stamp).seconds()) < std::abs((b.first - stamp).seconds());
            });

            const double dt = std::abs((best->first - stamp).seconds());
            if (dt > 0.1)
            {
                RCLCPP_WARN(get_logger(), "Marker %d: no frame within 0.1 s (minimum offset %.0f ms; buffer from %.3f to %.3f s, marker at %.3f s)",
                            msg->marker_id, dt * 1000.0, frames_.front().first.seconds(), frames_.back().first.seconds(), stamp.seconds());
                return;
            }
            RCLCPP_INFO(get_logger(), "Marker %d: frame chosen with offset %.0f ms", msg->marker_id, dt * 1000.0);

            cv::Mat img = best->second.clone();
            cv::Point c(static_cast<int>(msg->u), static_cast<int>(msg->v));
            int r = static_cast<int>(msg->s / 2.0F);

            cv::circle(img, c, r, cv::Scalar(0, 255, 0), 3);
            cv::putText(img, "ID: " + std::to_string(msg->marker_id),
                        cv::Point(c.x, std::max(20, c.y - r - 10)), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            const std::string path = out_dir_ + "/marker_" + std::to_string(msg->marker_id) + ".png";
            if (cv::imwrite(path, img))
                RCLCPP_INFO(get_logger(), "Image saved to %s", path.c_str());
            else
                RCLCPP_ERROR(get_logger(), "Unable to write %s (missing directory or no permissions?)", path.c_str());

            img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, img).toImageMsg());
            RCLCPP_INFO(get_logger(), "Annotated image of marker %d published on /marker_image", msg->marker_id);
        }
    };
} 
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::ImageAnnotator)
