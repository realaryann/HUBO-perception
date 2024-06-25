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
        pcl::PCLPointCloud2 cloud;
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(initial_cloud, cloud);
        *cloud_filtered = cloud;
        // do euclidean grouping

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_conversion(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filtered_conversion);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_parsed(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (auto pt : cloud_filtered_conversion->points) {
            float distance2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (pt.z < get_parameter("TABLE_HEIGHT").as_double() && distance2 < MAX_DIST2)
                cloud_filtered_parsed->push_back(pt);
        }

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(cloud_filtered_parsed);

        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        std::vector<pcl::PointIndices> cluster_indices;
        ec.setClusterTolerance (0.005); // 2cm
        ec.setMinClusterSize (get_parameter("MIN_CLUSTER_SIZE").as_int());
        ec.setMaxClusterSize (get_parameter("MAX_CLUSTER_SIZE").as_int());
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered_parsed);
        ec.extract (cluster_indices);
        // log new points
            // RCLCPP_INFO(this->get_logger(), "\nBEGIN --------------------");
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr first_cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // double sum_heights = 0;
        // for(auto cluster : cluster_indices)
        //     for (auto point : cluster.indices)
        //         sum_heights += (*cloud_filtered_conversion)[point].z;
        // double avg_z = sum_heights / cluster_indices
        for (size_t cluster = 0; cluster < cluster_indices.size(); cluster++) {
            pcl::PointXYZRGB cluster_center_pt;
            cluster_center_pt.x = 0; 
            cluster_center_pt.y = 0; 
            cluster_center_pt.z = 0;
            // RCLCPP_INFO(this->get_logger(), "point: (%lf, %lf, %lf)", cluster_center_pt.x, cluster_center_pt.y, cluster_center_pt.z);
            double red = std::rand() % 256;
            double green = std::rand() % 256;
            double blue = std::rand() % 256;
            cluster_center_pt.r = red;
            cluster_center_pt.g = green;
            cluster_center_pt.b = blue;
            cluster_center_pt.a = 0.05;
            for(auto point : cluster_indices[cluster].indices) {
                // RCLCPP_INFO(this->get_logger(), "z:%lf", (*cloud_filtered_conversion)[point].z);
                cluster_center_pt.x += cloud_filtered_parsed->points[point].x;
                cluster_center_pt.y += cloud_filtered_parsed->points[point].y;
                cluster_center_pt.z += cloud_filtered_parsed->points[point].z;
            }
            cluster_center_pt.x /= cluster_indices[cluster].indices.size();
            cluster_center_pt.y /= cluster_indices[cluster].indices.size();
            cluster_center_pt.z /= cluster_indices[cluster].indices.size();
            first_cluster_cloud->push_back(cluster_center_pt);
            // RCLCPP_INFO(this->get_logger(), "point: (%lf, %lf, %lf)", cluster_center_pt.x, cluster_center_pt.y, cluster_center_pt.z);
        }

        // planar segmentation

        // RCLCPP_INFO(this->get_logger(), "END --------------------\n");
        first_cluster_cloud->width = first_cluster_cloud->size();
        first_cluster_cloud->height = 1;
        first_cluster_cloud->is_dense = true;
        first_cluster_cloud->header.frame_id = cloud_filtered_conversion->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*first_cluster_cloud, msg);
        _publisher->publish(msg);
        

        // do later
        // convert back
        // publish the edited cloud
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
