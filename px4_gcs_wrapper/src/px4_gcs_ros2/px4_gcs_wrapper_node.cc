#include "px4_gcs_ros2/px4_gcs_wrapper.h"

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto px4_gcs_node = std::make_shared<px4_gcs::Px4GcsWrapper>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(px4_gcs_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}