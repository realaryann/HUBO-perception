#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

class Pipe : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

    void _on_subscriber(sensor_msgs::msg::PointCloud2 cloud) {
        cloud.header.frame_id = "camera_link";
        std::vector<sensor_msgs::msg::PointCloud2> clusters;
        // convert to pcl cloud
        pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(cloud, *cloud_pcl);
        // do euclidean grouping

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conversion(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_conversion);
        
        pcl::VoxelGrid<pcl::PointXYZ> downsampling;
        downsampling.setInputCloud(cloud_conversion);
        downsampling.setLeafSize(0.005, 0.005, 0.005);
        downsampling.filter(*cloud_conversion);

        cloud_conversion->width = cloud_conversion->size();
        cloud_conversion->height = 1;
        cloud_conversion->is_dense = true;
        cloud_conversion->header.frame_id = cloud.header.frame_id;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud_conversion, msg);;
        _publisher->publish(msg);
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