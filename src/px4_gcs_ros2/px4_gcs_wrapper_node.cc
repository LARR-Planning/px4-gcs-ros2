#include "px4_gcs_ros2/px4_gcs_wrapper.h"

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Px4GcsWrapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}