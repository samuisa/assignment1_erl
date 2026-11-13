#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include "assignment_msg/msg/aruco_info.hpp"

namespace assignment1_erl
{
    /**
     * @brief Converts raw ArUco detections into image-space marker measurements.
     *
     * Camera calibration and detected marker poses are used to estimate each
     * marker's horizontal pixel location and apparent size.
     */
    class DetectionManager : public rclcpp::Node
    {
    public:
        /**
         * @brief Creates the detection node and validates its projection parameters.
         * @param options ROS 2 node options supplied by the component container.
         */
        explicit DetectionManager(const rclcpp::NodeOptions &options)
            : Node("detection_node", options)
        {
            marker_size_m_ = declare_parameter<double>("marker_size_m", 0.2);
            desired_size_sim_px_ = declare_parameter<double>("desired_size_sim_px", 166.0);
            simulation_focal_length_px_ =
                declare_parameter<double>("simulation_focal_length_px", 381.4);
            image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
            marker_image_topic_ =
                declare_parameter<std::string>("marker_image_topic", "/marker_image");
            marker_reached_topic_ =
                declare_parameter<std::string>("marker_reached_topic", "/marker_reached");

            if (!std::isfinite(marker_size_m_) || marker_size_m_ <= 0.0 ||
                !std::isfinite(desired_size_sim_px_) || desired_size_sim_px_ <= 0.0 ||
                !std::isfinite(simulation_focal_length_px_) || simulation_focal_length_px_ <= 0.0)
            {
                throw std::invalid_argument("Marker size and camera focal lengths must be positive.");
            }

            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", 10,
                std::bind(&DetectionManager::camera_info_callback, this, std::placeholders::_1));

            aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections", 10,
                std::bind(&DetectionManager::aruco_callback, this, std::placeholders::_1));

            aruco_info_pub_ = this->create_publisher<assignment_msg::msg::ArucoInfo>("/aruco_info", 10);

            image_sub_ = image_transport::create_subscription(
                this, image_topic_,
                std::bind(&DetectionManager::image_callback, this, std::placeholders::_1),
                "raw");

            marker_image_pub_ = image_transport::create_publisher(this, marker_image_topic_);

            marker_reached_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                marker_reached_topic_, 10,
                std::bind(&DetectionManager::marker_reached_callback, this, std::placeholders::_1));
        }

    private:
        /** @brief Last known pixel-space position and apparent size of a marker. */
        struct MarkerPixelInfo
        {
            double u = 0.0;
            double v = 0.0;
            double s = 0.0;
            rclcpp::Time stamp;
        };

        static constexpr double kMarkerInfoMaxAgeSec = 0.5; ///< Discard marker pixel data older than this.

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr marker_reached_sub_;
        rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr aruco_info_pub_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher marker_image_pub_;

        double f_ = 0.0;                         ///< Camera horizontal focal length in pixels.
        double fy_ = 0.0;                        ///< Camera vertical focal length in pixels.
        double cx_ = 0.0;                        ///< Horizontal camera principal point in pixels.
        double cy_ = 0.0;                        ///< Vertical camera principal point in pixels.
        double marker_size_m_{};                 ///< Physical marker size used for range estimation.
        double desired_size_sim_px_{};            ///< Target apparent size in the reference simulation.
        double simulation_focal_length_px_{};     ///< Reference simulation focal length in pixels.
        bool camera_info_received_ = false;       ///< True once valid camera intrinsics are available.

        std::string image_topic_;                 ///< Raw camera image topic.
        std::string marker_image_topic_;           ///< Annotated marker image topic.
        std::string marker_reached_topic_;         ///< Topic signaling that a marker has been reached.

        sensor_msgs::msg::Image::ConstSharedPtr last_image_msg_;  ///< Most recent raw camera frame.
        std::map<int, MarkerPixelInfo> last_seen_markers_;        ///< Last pixel position per marker id.

        /**
         * @brief Stores valid camera focal length and horizontal principal point.
         * @param msg Camera calibration data.
         */
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
        {
            if (std::isfinite(msg->k[0]) && msg->k[0] > 0.0 &&
                std::isfinite(msg->k[2]) &&
                std::isfinite(msg->k[4]) && msg->k[4] > 0.0 &&
                std::isfinite(msg->k[5]))
            {
                f_ = msg->k[0];
                cx_ = msg->k[2];
                fy_ = msg->k[4];
                cy_ = msg->k[5];
                camera_info_received_ = true;
            }
        }

        /**
         * @brief Caches the latest raw camera frame for later annotation.
         * @param msg Raw camera image.
         */
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            last_image_msg_ = msg;
        }

        /**
         * @brief Projects each valid marker pose into image coordinates and publishes it.
         * @param msg ArUco detections containing marker IDs and camera-relative poses.
         */
        void aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg)
        {
            if (!camera_info_received_ || msg->markers.empty())
                return;

            assignment_msg::msg::ArucoInfo info_msg;

            for (const auto &marker : msg->markers)
            {
                float aruco_x = marker.pose.position.x;
                float aruco_y = marker.pose.position.y;
                float aruco_z = marker.pose.position.z;

                if (aruco_z > 0.0)
                {
                    info_msg.marker_id = marker.marker_id;
                    info_msg.u = static_cast<float>(f_ * (aruco_x / aruco_z) + cx_);
                    info_msg.s = static_cast<float>(f_ * marker_size_m_ / aruco_z);
                    info_msg.image_center_x = static_cast<float>(cx_);
                    info_msg.desired_size =
                        static_cast<float>(desired_size_sim_px_ * f_ / simulation_focal_length_px_);

                    const double v = fy_ * (aruco_y / aruco_z) + cy_;
                    last_seen_markers_[marker.marker_id] =
                        MarkerPixelInfo{info_msg.u, v, info_msg.s, this->now()};

                    aruco_info_pub_->publish(info_msg);
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Marker %d detected: u=%.1f px, size=%.1f px (target size=%.1f px).",
                        static_cast<int>(info_msg.marker_id), info_msg.u, info_msg.s,
                        info_msg.desired_size);
                }
            }
        }

        /**
         * @brief Draws a circle and id label on the cached frame and publishes it.
         * @param msg Id of the marker that has just been reached.
         */
        void marker_reached_callback(const std_msgs::msg::Int32::ConstSharedPtr msg)
        {
            if (!last_image_msg_)
            {
                RCLCPP_WARN(
                    get_logger(), "Marker %d reached but no camera frame has been received yet.",
                    msg->data);
                return;
            }

            const auto it = last_seen_markers_.find(msg->data);
            if (it == last_seen_markers_.end())
            {
                RCLCPP_WARN(
                    get_logger(), "Marker %d reached but its pixel position is unknown.", msg->data);
                return;
            }

            const double age_sec = (this->now() - it->second.stamp).seconds();
            if (age_sec > kMarkerInfoMaxAgeSec)
            {
                RCLCPP_WARN(
                    get_logger(), "Marker %d reached but its last known position is %.2f s old.",
                    msg->data, age_sec);
                return;
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(last_image_msg_, "bgr8");
            }
            catch (const cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
                return;
            }

            cv::Mat annotated = cv_ptr->image.clone();
            const MarkerPixelInfo &marker_info = it->second;
            const cv::Point center(
                static_cast<int>(std::lround(marker_info.u)),
                static_cast<int>(std::lround(marker_info.v)));
            const int radius = std::max(1, static_cast<int>(std::lround(marker_info.s / 2.0)));

            cv::circle(annotated, center, radius, cv::Scalar(0, 255, 0), 3);
            cv::putText(
                annotated, "id " + std::to_string(msg->data),
                cv::Point(center.x - radius, center.y - radius - 10),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            auto out_msg = cv_bridge::CvImage(last_image_msg_->header, "bgr8", annotated).toImageMsg();
            marker_image_pub_.publish(out_msg);

            RCLCPP_INFO(get_logger(), "Published annotated image for marker %d.", msg->data);
        }
    };
} // namespace assignment1_erl

RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::DetectionManager)