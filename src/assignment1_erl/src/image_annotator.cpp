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
    class ImageAnnotator : public rclcpp::Node
    {
    public:
        explicit ImageAnnotator(const rclcpp::NodeOptions &options) : Node("image_annotator", options)
        {
            out_dir_ = declare_parameter<std::string>("output_dir", "marker_images");
            // Topic base dell'immagine (senza /compressed), relativo al namespace del nodo.
            // Simulazione: "camera/image"; robot reale: "camera/rgb/image_raw".
            const std::string image_topic = declare_parameter<std::string>("image_topic", "camera/image");
            // true: si iscrive a <image_topic>/compressed; false: si iscrive al topic raw.
            const bool compressed = declare_parameter<bool>("image_compressed", true);

            // imwrite non crea le cartelle e fallisce in silenzio se non esistono.
            std::error_code ec;
            std::filesystem::create_directories(out_dir_, ec);
            if (ec)
                RCLCPP_WARN(get_logger(), "Impossibile creare la cartella %s: %s", out_dir_.c_str(), ec.message().c_str());

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

            // Se le immagini non arrivano lo dice: tipicamente image_topic/image_compressed sbagliati, namespace o dominio
            last_frame_ = now();
            watchdog_ = create_wall_timer(std::chrono::seconds(1), [this]() {
                const double age = (now() - last_frame_).seconds();
                if (age > 3.0)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "Nessun frame su %s da %.1f s: controlla image_topic, image_compressed, namespace e dominio",
                                         in_topic_.c_str(), age);
            });

            RCLCPP_INFO(get_logger(), "Annotator avviato | immagini: %s (%s) | cartella di output: %s",
                        in_topic_.c_str(), compressed ? "compressed" : "raw", out_dir_.c_str());
            RCLCPP_INFO(get_logger(), "Ascolto i marker raggiunti su %s", marker_sub_->get_topic_name());
        }

    private:
        std::string out_dir_, in_topic_;
        std::deque<std::pair<rclcpp::Time, cv::Mat>> frames_;
        rclcpp::Time last_frame_;
        bool got_frame_ = false;
        rclcpp::TimerBase::SharedPtr watchdog_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr comp_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
        rclcpp::Subscription<assignment_msg::msg::ArucoInfo>::SharedPtr marker_sub_;
        image_transport::Publisher img_pub_;

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
                    RCLCPP_INFO(get_logger(), "Primo frame ricevuto su %s: %dx%d", in_topic_.c_str(), cv_ptr->image.cols, cv_ptr->image.rows);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Frame non decodificato: %s", e.what());
            }
        }

        void marker_cb(const assignment_msg::msg::ArucoInfo::ConstSharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Marker %d raggiunto (u=%.0f v=%.0f s=%.0f): cerco il frame piu' vicino, buffer=%zu frame",
                        msg->marker_id, msg->u, msg->v, msg->s, frames_.size());

            if (frames_.empty())
            {
                RCLCPP_WARN(get_logger(), "Marker %d raggiunto ma nessun frame ricevuto: controlla image_topic e image_compressed", msg->marker_id);
                return;
            }

            rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
            auto best = std::min_element(frames_.begin(), frames_.end(), [&](const auto &a, const auto &b) {
                return std::abs((a.first - stamp).seconds()) < std::abs((b.first - stamp).seconds());
            });

            const double dt = std::abs((best->first - stamp).seconds());
            if (dt > 0.1)
            {
                RCLCPP_WARN(get_logger(), "Marker %d: nessun frame entro 0.1 s (scarto minimo %.0f ms; buffer da %.3f a %.3f s, marker a %.3f s)",
                            msg->marker_id, dt * 1000.0, frames_.front().first.seconds(), frames_.back().first.seconds(), stamp.seconds());
                return;
            }
            RCLCPP_INFO(get_logger(), "Marker %d: frame scelto con scarto %.0f ms", msg->marker_id, dt * 1000.0);

            cv::Mat img = best->second.clone();
            cv::Point c(static_cast<int>(msg->u), static_cast<int>(msg->v));
            int r = static_cast<int>(msg->s / 2.0F);

            cv::circle(img, c, r, cv::Scalar(0, 255, 0), 3);
            cv::putText(img, "ID: " + std::to_string(msg->marker_id),
                        cv::Point(c.x, std::max(20, c.y - r - 10)), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            const std::string path = out_dir_ + "/marker_" + std::to_string(msg->marker_id) + ".png";
            if (cv::imwrite(path, img))
                RCLCPP_INFO(get_logger(), "Immagine salvata in %s", path.c_str());
            else
                RCLCPP_ERROR(get_logger(), "Impossibile scrivere %s (cartella inesistente o senza permessi?)", path.c_str());

            img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, img).toImageMsg());
            RCLCPP_INFO(get_logger(), "Immagine annotata del marker %d pubblicata su /marker_image", msg->marker_id);
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::ImageAnnotator)