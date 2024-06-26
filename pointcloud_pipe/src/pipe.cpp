#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Pipe : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

    void _on_subscriber(sensor_msgs::msg::PointCloud2 cloud) {
        _publisher->publish(cloud);
    }
public:
    Pipe() : Node("pipe") {
        _subscriber = create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/points", rclcpp::SensorDataQoS(), std::bind(&Pipe::_on_subscriber, this, std::placeholders::_1));
        _publisher = create_publisher<sensor_msgs::msg::PointCloud2>("piped_pointcloud", 10);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pipe>());
    rclcpp::shutdown();
    return 0;
}