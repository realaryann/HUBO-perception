#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

//0.674361 -0.001074 1.25002 0 -0 3.14

class Broadcaster : public rclcpp::Node {
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> _broadcaster;
    rclcpp::TimerBase::SharedPtr _timer;

    void _on_timer() {
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "world";
        t.child_frame_id = "camera_link";
        t.transform.translation.x = 0.674361;
        t.transform.translation.y = -0.001074;
        t.transform.translation.z = 1.250020;
        t.transform.rotation.x = 0.9996;
        t.transform.rotation.w = 0.0273;
        _broadcaster->sendTransform(t);
    }
public:
    Broadcaster() : Node("sim_transform_broadcaster") {
        _broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        _timer = create_wall_timer(5s, std::bind(&Broadcaster::_on_timer, this));
    }
}; // class Broadcaster

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Broadcaster>());
    rclcpp::shutdown();
    return 0;
}