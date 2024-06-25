#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/search/kdtree.h"

#include <cstdlib>
// Run the following to get point cloud information
// gazebo /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_depth_camera_demo.world

const std::string SUB_TOPIC = "demo_cam/mycamera/points_demo";
const double MAX_DIST = 4.0;
const double MAX_DIST2 = MAX_DIST * MAX_DIST;
const double FLOOR_HEIGHT = 3;

class PointCloudParser : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;

    void _on_subscriber(sensor_msgs::msg::PointCloud2 initial_cloud) {
        // RCLCPP_INFO(this->get_logger(), "STARTING");
        std::vector<sensor_msgs::msg::PointCloud2> clusters;
        // convert to pcl cloud
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(initial_cloud, *cloud);
        // do euclidean grouping

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromPCLPointCloud2(*cloud, *cloud_conversion);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_parsed(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (auto pt : cloud_conversion->points) {
            float distance2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (pt.z < get_parameter("TABLE_HEIGHT").as_double() && distance2 < MAX_DIST2)
                cloud_parsed->push_back(pt);
        }

        // TODO: Do negative of table plane, publish object tfs

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // RCLCPP_INFO(this->get_logger(), "END --------------------\n");
        object_cluster_cloud->width = object_cluster_cloud->size();
        object_cluster_cloud->height = 1;
        object_cluster_cloud->is_dense = true;
        object_cluster_cloud->header.frame_id = cloud_conversion->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*object_cluster_cloud, msg);
        _publisher->publish(msg);
    }
public: 
    PointCloudParser() : Node("pointcloud_parser") {
        _subscriber = create_subscription<sensor_msgs::msg::PointCloud2>(SUB_TOPIC, 10, std::bind(&PointCloudParser::_on_subscriber, this, std::placeholders::_1));
        _publisher = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", 10);
        declare_parameter<int>("MIN_CLUSTER_SIZE");
        declare_parameter<int>("MAX_CLUSTER_SIZE");
        declare_parameter<double>("TABLE_HEIGHT");
        set_parameter(rclcpp::Parameter("MIN_CLUSTER_SIZE", 600));
        set_parameter(rclcpp::Parameter("MAX_CLUSTER_SIZE", 5000));
        set_parameter(rclcpp::Parameter("TABLE_HEIGHT", 3.0));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudParser>());
    rclcpp::shutdown();
    return 0;
}
