/**
 * @file image_detection.cpp
 * @brief Node that converts the ArUco detections into image-space measurements of the target marker.
 *
 * It receives the 3D pose of the markers estimated by the aruco tracker, keeps only the
 * marker the state machine is currently looking for, and computes where it appears in the
 * image (pixel coordinates u, v), how big it appears (side s in pixels) and the size it
 * should have when the robot is at the desired stopping distance. The result is
 * published on /aruco_info.
 */
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include "assignment_msg/msg/aruco_info.hpp"
#include "assignment_msg/msg/fsm_status.hpp"

namespace assignment1_erl
{
    /**
     * @class DetectionManager
     * @brief Robot Operating System 2 (ROS 2) node that projects the detected target marker onto the image plane.
     *
     * Parameters:
     *  - desired_size_sim_px        (double): marker side in pixels at the stopping distance, in simulation
     *  - simulation_focal_length_px (double): focal length in pixels used in simulation (reference value)
     *  - marker_size                (double): real side of the marker in meters (must match the tracker)
     *
     * Subscribes to: /camera/camera_info, /fsm_status, /aruco_detections.
     * Publishes: /aruco_info.
     */
    class DetectionManager : public rclcpp::Node
    {
    public:
        /**
         * @brief Reads the parameters, creates subscribers/publisher and a watchdog
         *        that warns if the tracker stops publishing.
         * @param options Node options (needed to load the node as a component).
         */
        explicit DetectionManager(const rclcpp::NodeOptions &options) : Node("detection_node", options)
        {
            // Dimensione (pixel) del marker alla distanza voluta, misurata con la focale di riferimento.
            // La dimensione effettiva viene scalata con la focale della camera reale (vedi aruco_cb).
            desired_sim_px_ = declare_parameter<double>("desired_size_sim_px", 140.0);
            ref_focal_px_ = declare_parameter<double>("simulation_focal_length_px", 381.4);
            // Lato reale del marker (m): deve coincidere con marker_size dell'aruco_tracker.
            marker_size_ = declare_parameter<double>("marker_size", 0.2);

            // Camera intrinsics callback: stores focal length (fx) and principal point (cx, cy)
            cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/camera_info", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                    if (msg->k[0] > 0.0)
                    {
                        const bool first = (f_ <= 0.0);
                        f_ = msg->k[0]; cx_ = msg->k[2]; cy_ = msg->k[5];
                        if (first)
                        {
                            const double desired = desired_sim_px_ * f_ / ref_focal_px_;
                            RCLCPP_INFO(get_logger(),
                                        "Camera info received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f | image %ux%u | distortion coefficients: %zu",
                                        msg->k[0], msg->k[4], cx_, cy_, msg->width, msg->height, msg->d.size());
                            RCLCPP_INFO(get_logger(),
                                        "effective desired_size = %.1f px (%.1f px at focal length %.1f, scaled with fx=%.1f) -> stopping distance ~ %.2f m",
                                        desired, desired_sim_px_, ref_focal_px_, f_, marker_size_ * ref_focal_px_ / desired_sim_px_);
                        }
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                             "camera_info with fx=0 (null K matrix: camera not calibrated?): without K nothing can be computed");
                    }
                });

            // State machine callback: keeps track of which marker is the current target
            fsm_sub_ = create_subscription<assignment_msg::msg::FsmStatus>(
                "/fsm_status", 10,
                [this](const assignment_msg::msg::FsmStatus::ConstSharedPtr msg) {
                    if (msg->target_id != target_id_)
                        RCLCPP_INFO(get_logger(), "Target changed: marker %d -> marker %d", target_id_, static_cast<int>(msg->target_id));
                    target_id_ = msg->target_id;
                });

            aruco_sub_ = create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections", 10,
                [this](const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg) { aruco_cb(msg); });

            pub_ = create_publisher<assignment_msg::msg::ArucoInfo>("/aruco_info", 10);

            // Se il tracker non pubblica nulla, lo dice: tipicamente immagine (raw/compressed), camera_info, namespace o dominio sbagliati
            last_det_ = now();
            watchdog_ = create_wall_timer(std::chrono::seconds(1), [this]() {
                const double age = (now() - last_det_).seconds();
                if (age > 3.0)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "No message on %s for %.1f s: tracker off, or images/camera_info not arriving (compressed or raw? namespace? domain?)",
                                         aruco_sub_->get_topic_name(), age);
            });

            RCLCPP_INFO(get_logger(), "Parameters: desired_size_sim_px=%.1f, simulation_focal_length_px=%.1f, marker_size=%.3f m",
                        desired_sim_px_, ref_focal_px_, marker_size_);
            RCLCPP_INFO(get_logger(), "Listening on: %s, %s, %s", cam_sub_->get_topic_name(), fsm_sub_->get_topic_name(), aruco_sub_->get_topic_name());
            RCLCPP_INFO(get_logger(), "Publishing on: %s | initial target = marker %d", pub_->get_topic_name(), target_id_);
        }

    private:
        double f_ = 0.0, cx_ = 0.0, cy_ = 0.0;                                   ///< Camera focal length and principal point (pixels)
        double desired_sim_px_ = 140.0, ref_focal_px_ = 381.4, marker_size_ = 0.2; ///< Node parameters (see class description)
        int target_id_ = 1;                                                      ///< Marker currently searched by the state machine
        rclcpp::Time last_det_;                                                  ///< Time of the last detection message (used by the watchdog)
        rclcpp::TimerBase::SharedPtr watchdog_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
        rclcpp::Subscription<assignment_msg::msg::FsmStatus>::SharedPtr fsm_sub_;
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
        rclcpp::Publisher<assignment_msg::msg::ArucoInfo>::SharedPtr pub_;

        /**
         * @brief Callback of /aruco_detections: computes and publishes the image-space data of the target marker.
         * @param msg List of the markers detected in the current image, with their 3D pose.
         *
         * For the target marker (other markers are ignored) it uses the pinhole camera model:
         *  - u = fx * x / z + cx and v = fy * y / z + cy (here fx is used for both): pixel position of the marker center
         *  - s = fx * L / z: marker side in pixels, where L is the real marker side and z its distance
         *  - desired_size: size the marker must have at the stopping distance, scaled with the real camera focal length
         * Nothing is published if the camera info has not been received yet.
         */
        void aruco_cb(const aruco_opencv_msgs::msg::ArucoDetection::ConstSharedPtr msg)
        {
            last_det_ = now();

            // Senza la camera info (focale nulla) non si puo' calcolare nulla
            if (f_ <= 0.0)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Detections received but camera info missing (null focal length): no /aruco_info published");
                return;
            }

            bool found = false;
            std::string ids;

            for (const auto &m : msg->markers)
            {
                ids += std::to_string(m.marker_id) + " ";

                // Ignora completamente qualsiasi marker diverso dal target attuale
                if (m.marker_id != target_id_ || m.pose.position.z <= 0.0) continue;

                assignment_msg::msg::ArucoInfo info;
                info.header = msg->header;
                info.marker_id = m.marker_id;
                info.u = static_cast<float>(f_ * (m.pose.position.x / m.pose.position.z) + cx_);
                info.v = static_cast<float>(f_ * (m.pose.position.y / m.pose.position.z) + cy_);
                // Lato del marker in pixel: s = f * L / z
                info.s = static_cast<float>(f_ * marker_size_ / m.pose.position.z);
                info.image_center_x = static_cast<float>(cx_);
                // Stessa distanza fisica della simulazione: la dimensione in pixel scala con la focale
                info.desired_size = static_cast<float>(desired_sim_px_ * f_ / ref_focal_px_);
                pub_->publish(info);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                     "Marker %d: u=%.1f v=%.1f s=%.1f px (desired=%.1f) | pose: x=%.2f z=%.2f m",
                                     m.marker_id, info.u, info.v, info.s, info.desired_size,
                                     m.pose.position.x, m.pose.position.z);
                found = true;
                break; // Trovato il target, esce subito dal ciclo
            }

            if (!found)
            {
                if (msg->markers.empty())
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                         "Tracker active but no marker in the image (looking for marker %d)", target_id_);
                else
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Visible markers: [ %s] but the target is marker %d: ignored", ids.c_str(), target_id_);
            }
        }
    };
} // namespace assignment1_erl
RCLCPP_COMPONENTS_REGISTER_NODE(assignment1_erl::DetectionManager)
