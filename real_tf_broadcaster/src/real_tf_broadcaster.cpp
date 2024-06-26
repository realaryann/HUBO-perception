#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"


using namespace std::chrono_literals;

//0.674361 -0.001074 1.25002 0 -0 3.14
// ros2 run tf2_ros static_transform_publisher 0.67 -0.001 1.25 0 0 -1.57 world camera_link

class Broadcaster : public rclcpp::Node {
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _broadcaster;
    rclcpp::TimerBase::SharedPtr _timer;

public:
    Broadcaster() : Node("real_transform_broadcaster") {
        _broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "camera_link";
        t.transform.translation.z = 2.0;
        // For rotation, some factor of pi/2 - angle helps
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.8959;
        t.transform.rotation.z = -0.4443;
        t.transform.rotation.w = 0.0;
        _broadcaster->sendTransform(t);  
    }
}; // class Broadcaster

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Broadcaster>());
    rclcpp::shutdown();
    return 0;
}