#include "px4_gcs_ros2/px4_gcs_wrapper.h"
#include <eigen3/Eigen/Core>

px4_gcs::Px4GcsWrapper::Px4GcsWrapper() : Node("px4_gcs_wrapper") {}

px4_gcs::Px4GcsWrapper::Px4GcsWrapper(const rclcpp::NodeOptions &options_input)
    : Node("px4_gcs_wrapper", options_input) {
  // Subscriber
  rclcpp::SubscriptionOptions options;
  { // subscriber group
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    current_pose_external_subscriber_ = create_subscription<LocalPoseExternal>(
        "~/current_external_pose", rclcpp::QoS(1),
        std::bind(&Px4GcsWrapper::ExternalPoseCallback, this, std::placeholders::_1), options);
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    current_pose_rtps_subscriber_ = create_subscription<LocalPoseRtps>(
        "~/current_rtps_pose", rclcpp::QoS(1), std::bind(&Px4GcsWrapper::RtpsPoseCallback, this, std::placeholders::_1),
        options);
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    goal_pose_subscriber_ = create_subscription<LocalGoalPose>(
        "~/local_goal_pose", rclcpp::QoS(1), std::bind(&Px4GcsWrapper::GoalPoseCallback, this, std::placeholders::_1),
        options);
  }
  // Server
  init_home_server_ = this->create_service<InitHomeService>(
      "~/init_home", std::bind(&Px4GcsWrapper::InitHomeCallback, this, std::placeholders::_1, std::placeholders::_2));
  keyboard_input_server_ = this->create_service<KeyboardInputService>(
      "~/keyboard_input",
      std::bind(&Px4GcsWrapper::KeyboardInputCallback, this, std::placeholders::_1, std::placeholders::_2));
  switch_mode_server_ =
      this->create_service<SwitchModeService>("~/switch_mode", std::bind(&Px4GcsWrapper::SwitchModeCallback, this,
                                                                         std::placeholders::_1, std::placeholders::_2));

  // Publisher
  publisher_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  publisher_timer_ = this->create_wall_timer(30ms,std::bind(&Px4GcsWrapper::PublisherTimerCallback,this),publisher_callback_group_);
  publisher_.external_pose_publisher = create_publisher<ExternalPoseVis>("~/external_pose",rclcpp::QoS(1));
  publisher_.rtps_pose_publisher = create_publisher<RtpsPoseVis>("~/local_current_pose",rclcpp::QoS(1));
  publisher_.local_goal_pose_publisher = create_publisher<LocalGoalPose>("~/local_goal_pose",rclcpp::QoS(1));
}

bool px4_gcs::Px4GcsWrapper::mav_init() {
  if (is_tf_received_ and is_rtps_pose_received_) {
    pose_des_keyboard_ = pose_rtps_;
    if (abs(pose_rtps_.pose.position.x) < 1e4) {
      is_init_mav_ = true;
      RCLCPP_INFO_STREAM_ONCE(get_logger(), "[PX4 GCS Wrapper] Initializing the homing point with the current pose.");
      return true;
    } else {
      is_init_mav_ = false;
      RCLCPP_INFO_STREAM_ONCE(get_logger(), "[PX4 GCS Wrapper] Tried to initialize homing point from rtps local "
                                            "position. But the value sanity check (x<1e4) failed.");
    }
  } else {
    RCLCPP_WARN_STREAM_SKIPFIRST(
        get_logger(),
        "[PX4 GCS Wrapper] Initialization tried but no pose information from either micro_rtps or external sources");
    return false;
  }
}

bool px4_gcs::Px4GcsWrapper::InitHomeCallback(const std::shared_ptr<InitHomeService::Request> request,
                                              std::shared_ptr<InitHomeService::Response> response) {
  response->is_success = mav_init();
  return true;
}

bool px4_gcs::Px4GcsWrapper::KeyboardInputCallback(const std::shared_ptr<KeyboardInputService::Request> request,
                                                   std::shared_ptr<KeyboardInputService::Response> response) {
  std::string keyinput = request->key;
  bool is_success = false;
  // double increment_xyz = 0.05;
  // double increment_yaw = 3.141592 / 8; // maybe to be tuned
  Eigen::Vector4d move_pose;
  move_pose.setZero();

  // prepare delta pose
  switch (tolower(keyinput.c_str()[0])) {
  case 'o': {
    increment_xyz_ -= 0.02;
    RCLCPP_DEBUG(get_logger(), "Decreased linear step. current: %f", increment_xyz_);
    break;
  }
  case 'p': {
    increment_xyz_ += 0.02;
    RCLCPP_DEBUG(get_logger(), "Increased linear step. current: %f", increment_xyz_);
    break;
  }
  case 'k': {
    increment_yaw_ -= 3.141592 / 12;
    RCLCPP_DEBUG(get_logger(), "Decreased yaw step. current: %f", increment_yaw_);
    break;
  }
  case 'l': {
    increment_yaw_ += 3.141592 / 12;
    RCLCPP_DEBUG(get_logger(), "Increased yaw step. current: %f", increment_yaw_);
    break;
  }
  case 'w':
    move_pose(0) = increment_xyz_;
    break;
  case 's':
    move_pose(0) = -increment_xyz_;
    break;
  case 'a':
    move_pose(1) = -increment_xyz_;
    break;
  case 'd':
    move_pose(1) = increment_xyz_;
    break;
  case 'z':
    move_pose(2) = increment_xyz_;
    break;
  case 'c':
    move_pose(2) = -increment_xyz_;
    break;
  case 'q':
    move_pose(3) = -increment_yaw_;
    break;
  case 'e':
    move_pose(3) = increment_yaw_;
    break;
  }
  // apply delta pose
  // std::cout<<move_pose<<std::endl;
  //    return move_mav(move_pose(0), move_pose(1), move_pose(2), move_pose(3));
}

bool px4_gcs::Px4GcsWrapper::SwitchModeCallback(const std::shared_ptr<SwitchModeService::Request> request,
                                                std::shared_ptr<SwitchModeService::Response> response) {
  if (request->mode == 0) { // Keyboard Mode Requested
    if (mav_init()) {
      mode_ = 0;
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "[PX4 GCS Wrapper] Mav init was denied. Is is listening tf?");
      return false;
    }
  } else { // Planning Mode Requested
    if (is_planning_received_) {
      mode_ = 1;
      return true;
    } else
      return false;
  }
}

void px4_gcs::Px4GcsWrapper::ExternalPoseCallback(const LocalPoseExternal::SharedPtr msg) {
  // NWU to NED
  pose_external_.header.frame_id = "world_ned";
  pose_external_.header.stamp = msg->header.stamp;
  pose_external_.pose.position.x = msg->pose.position.x;
  pose_external_.pose.position.y = -msg->pose.position.y;
  pose_external_.pose.position.z = -msg->pose.position.z;
  pose_external_.pose.orientation.x = msg->pose.orientation.y;
  pose_external_.pose.orientation.y = msg->pose.orientation.x;
  pose_external_.pose.orientation.z = -msg->pose.orientation.z;
  pose_external_.pose.orientation.w = msg->pose.orientation.w;
  is_tf_received_ = true;
}

void px4_gcs::Px4GcsWrapper::RtpsPoseCallback(const LocalPoseRtps::SharedPtr msg) {
  // NED to NED
  pose_rtps_.header.frame_id = "world_ned";
  pose_rtps_.pose.position.x = msg->x;
  pose_rtps_.pose.position.y = msg->y;
  pose_rtps_.pose.position.z = msg->z;
  pose_rtps_.pose.orientation.x = msg->q[0];
  pose_rtps_.pose.orientation.y = msg->q[1];
  pose_rtps_.pose.orientation.z = msg->q[2];
  pose_rtps_.pose.orientation.w = msg->q[3];
  is_rtps_pose_received_ = true;
}

void px4_gcs::Px4GcsWrapper::GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // NWU to NED
  pose_des_planner_.header.stamp = msg->header.stamp;
  pose_des_planner_.header.frame_id = "world_ned";
  pose_des_planner_.pose.position.x = msg->pose.position.x;
  pose_des_planner_.pose.position.y = -msg->pose.position.y;
  pose_des_planner_.pose.position.z = -msg->pose.position.z;
  pose_des_planner_.pose.orientation.x = msg->pose.orientation.y;
  pose_des_planner_.pose.orientation.y = msg->pose.orientation.x;
  pose_des_planner_.pose.orientation.z = -msg->pose.orientation.z;
  pose_des_planner_.pose.orientation.w = msg->pose.orientation.w;
}

bool px4_gcs::Px4GcsWrapper::move_mav(const double &dx, const double &dy, const double &dz, const double &dyaw) {
  if (mode_ == 0 and is_init_mav_) {
    Eigen::Vector3d dpose(dx, dy, dz);
    tf2::Quaternion q_cur_des;
    q_cur_des.setX(pose_des_keyboard_.pose.orientation.x);
    q_cur_des.setY(pose_des_keyboard_.pose.orientation.y);
    q_cur_des.setZ(pose_des_keyboard_.pose.orientation.z);
    q_cur_des.setW(pose_des_keyboard_.pose.orientation.w);

    geometry_msgs::msg::TransformStamped transfrom_cur;
    transfrom_cur.transform.translation.x = 0.0;
    transfrom_cur.transform.translation.y = 0.0;
    transfrom_cur.transform.translation.z = 0.0;
    transfrom_cur.transform.rotation.x = q_cur_des.x();
    transfrom_cur.transform.rotation.y = q_cur_des.y();
    transfrom_cur.transform.rotation.z = q_cur_des.z();
    transfrom_cur.transform.rotation.w = q_cur_des.w();

    Eigen::Isometry3d Twb = tf2::transformToEigen(transfrom_cur);
    Eigen::Vector3d dpose_w = Twb * dpose;
    // Step 1: modify xyz
    pose_des_keyboard_.pose.position.x += dpose_w(0);
    pose_des_keyboard_.pose.position.y += dpose_w(1);
    pose_des_keyboard_.pose.position.z += dpose_w(2);
    // Step 2: modify yaw direcion
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_cur_des).getEulerYPR(yaw, pitch, roll);
    yaw += dyaw; // here, we adjust yaw
    tf2::Quaternion q_des = tf2::Quaternion();
    q_des.setRPY(roll, pitch, yaw);

    pose_des_keyboard_.pose.orientation.x = q_des.getX();
    pose_des_keyboard_.pose.orientation.y = q_des.getY();
    pose_des_keyboard_.pose.orientation.z = q_des.getZ();
    pose_des_keyboard_.pose.orientation.w = q_des.getW();
    return true;
  } else {
    if (not mode_ == 0)
      RCLCPP_WARN_STREAM_SKIPFIRST(get_logger(), "MAV receives planning setpoint. To enable keyboard, switch mode.");
    else
      RCLCPP_WARN_STREAM_SKIPFIRST(get_logger(), "Homing point not initialized.");
    return false;
  }
}
void px4_gcs::Px4GcsWrapper::PublisherTimerCallback() {
  { // local pose publisher

  }
  { // external pose publisher

  }
  { // local goal pose publisher

  }
}
