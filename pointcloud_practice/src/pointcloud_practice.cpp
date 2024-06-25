#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/search/kdtree.h"

#include "tf2_ros/transform_broadcaster.h"

#include <cstdlib>
// Run the following to get point cloud information
// gazebo /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_depth_camera_demo.world

const std::string SUB_TOPIC = "camera/ir/image";
const double MAX_DIST = 4.0;
const double MAX_DIST2 = MAX_DIST * MAX_DIST;
const double FLOOR_HEIGHT = 3;

class PointCloudParser : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _object_location_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_centers;

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
            double table_height = get_parameter("TABLE_HEIGHT").as_double();
            if (pt.z < table_height && distance2 < MAX_DIST2)
                cloud_parsed->push_back(pt);
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        *object_cluster_cloud = *cloud_parsed;
        // TODO: Do negative of table plane, publish object tfs
        // REMOVE FLOOR
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(object_cluster_cloud);
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(object_cluster_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_floor(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*removed_floor);

        // REMOVE TABLE
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(removed_floor);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(removed_floor);
        extract.setIndices(inliers);
        extract.setNegative(true);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_table(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*removed_table);

        //Extract clusters
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZRGB>);
        std::vector<pcl::PointIndices> cluster_vector;
        clusterer.setClusterTolerance(0.1);
        clusterer.setInputCloud(removed_table);
        clusterer.setMinClusterSize(get_parameter("MIN_CLUSTER_SIZE").as_int());
        clusterer.setMaxClusterSize(get_parameter("MAX_CLUSTER_SIZE").as_int());
        clusterer.setSearchMethod(search_method);

        clusterer.extract(cluster_vector);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_centers(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (auto cluster : cluster_vector) {
            pcl::PointXYZRGB center_point;
            center_point.r = rand() % 256;
            center_point.g = rand() % 256;
            center_point.b = rand() % 256;
            for (auto index : cluster.indices) {
                pcl::PointXYZRGB pt = removed_table->points[index];
                center_point.x += pt.x;
                center_point.y += pt.y;
                center_point.z += pt.z;
                pt.r = 255 - center_point.r;
                pt.g = 255 - center_point.g;
                pt.b = 255 - center_point.b;
                output_cloud->push_back(pt);
            }
            center_point.x /= cluster.indices.size();
            center_point.y /= cluster.indices.size();
            center_point.z /= cluster.indices.size();
            output_centers->push_back(center_point);
        }
        // Output final cloud
        output_cloud->width = output_cloud->size();
        output_cloud->height = 1;
        output_cloud->is_dense = true;
        output_cloud->header.frame_id = cloud_conversion->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*output_cloud, msg);
        _publisher->publish(msg);

        // Output final cloud
        for (size_t point  = 0; point < output_centers->points.size(); point++) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "camera_link";
            t.child_frame_id = "object_" + std::to_string(point);
            t.transform.translation.x = output_centers->points[point].x;
            t.transform.translation.y = output_centers->points[point].y;
            t.transform.translation.z = output_centers->points[point].z;
            // For rotation, some factor of pi/2 - angle helps
            _object_location_broadcaster->sendTransform(t);
        }
        output_centers->width = output_centers->size();
        output_centers->height = 1;
        output_centers->is_dense = true;
        output_centers->header.frame_id = cloud_conversion->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg_centers;
        pcl::toROSMsg(*output_centers, msg_centers);
        _publisher_centers->publish(msg_centers);
    }
public: 
    PointCloudParser() : Node("pointcloud_parser") {
        _subscriber = create_subscription<sensor_msgs::msg::PointCloud2>(SUB_TOPIC, 10, std::bind(&PointCloudParser::_on_subscriber, this, std::placeholders::_1));
        _publisher = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", 10);
        _object_location_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _publisher_centers = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_centers", 10);
        declare_parameter<int>("MIN_CLUSTER_SIZE");
        declare_parameter<int>("MAX_CLUSTER_SIZE");
        declare_parameter<double>("TABLE_HEIGHT");
        set_parameter(rclcpp::Parameter("MIN_CLUSTER_SIZE", 1));
        set_parameter(rclcpp::Parameter("MAX_CLUSTER_SIZE", 10000));
        set_parameter(rclcpp::Parameter("TABLE_HEIGHT", 2.1));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudParser>());
    rclcpp::shutdown();
    return 0;
}

// Ros2 open ni driver
