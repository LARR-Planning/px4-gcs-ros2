//
// Created by larr-gcs on 23. 8. 12.
//

#ifndef PX4_GCS_ROS2_PX4_GCS_ROS2_H
#define PX4_GCS_ROS2_PX4_GCS_ROS2_H

#include "rclcpp/rclcpp.hpp"
#include "px4_gcs_interfaces/srv/init_home.hpp"
#include "px4_gcs_interfaces/srv/keyboard_input.hpp"
#include "px4_gcs_interfaces/srv/switch_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"

#include <Eigen/Core>

using InitHomeService = px4_gcs_interfaces::srv::InitHome;
using InitHomeServer = rclcpp::Service<InitHomeService>::SharedPtr;
using KeyboardInputService = px4_gcs_interfaces::srv::KeyboardInput;
using KeyboardInputServer = rclcpp::Service<KeyboardInputService>::SharedPtr;
using SwitchModeService = px4_gcs_interfaces::srv::SwitchMode;
using SwitchModeServer = rclcpp::Service<SwitchModeService>::SharedPtr;

using LocalGoalPose = geometry_msgs::msg::PoseStamped;
using LocalPoseDefault = geometry_msgs::msg::PoseStamped;
using LocalPoseExternal = geometry_msgs::msg::PoseStamped;
using LocalPoseRtps = px4_msgs::msg::VehicleOdometry;

using CurrentPoseRtpsSubscriber = rclcpp::Subscription<LocalPoseRtps>::SharedPtr;
using CurrentPoseExternalSubscriber = rclcpp::Subscription<LocalPoseExternal>::SharedPtr;
using GoalPoseSubscriber = rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr;

namespace px4_gcs {
    class Px4GcsWrapper : public rclcpp::Node {
    private:
        bool is_tf_received_ = false;
        bool is_rtps_pose_received_ = false;
        bool is_init_mav_ = false;

        double increment_xyz_;
        double increment_yaw_;

        LocalGoalPose pose_des_keyboard_;
        LocalGoalPose pose_des_planner_;
        LocalPoseDefault pose_external_;
        LocalPoseDefault pose_rtps_;
        LocalPoseDefault pose_init_;

        unsigned int mode = 0;

        // Service
        InitHomeServer init_home_server_;
        KeyboardInputServer keyboard_input_server_;
        SwitchModeServer switch_mode_server_;
        bool InitHomeCallback(const std::shared_ptr<InitHomeService::Request> &request,
                              std::shared_ptr<InitHomeService::Response> &response);

        bool KeyboardInputCallback(const std::shared_ptr<KeyboardInputService::Request> &request,
                                   std::shared_ptr<KeyboardInputService::Response> &response);

        bool SwitchModeCallback(const std::shared_ptr<SwitchModeService::Request> &request,
                                std::shared_ptr<SwitchModeService::Response> &response);

        // mav initialization
        bool mav_init();
        bool move_mav(const double &dx, const double &dy, const double &dz, const double &dyaw);
        // Subscriber
        CurrentPoseRtpsSubscriber current_pose_rtps_subscriber_;
        CurrentPoseExternalSubscriber current_pose_external_subscriber_;
        GoalPoseSubscriber goal_pose_subscriber_;

        // Subscriber Callback
        void ExternalPoseCallback(const LocalPoseExternal::SharedPtr msg);
        void RtpsPoseCallback(const LocalPoseRtps::SharedPtr msg);
        void GoalPoseCallback(const LocalGoalPose::SharedPtr msg);
        // Publisher


    public:
        Px4GcsWrapper();
        explicit Px4GcsWrapper(const rclcpp::NodeOptions &options_input);



    };

}   // namespace px4_gcs

#endif //PX4_GCS_ROS2_PX4_GCS_ROS2_H
