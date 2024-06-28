#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "std_msgs/msg/string.hpp"
#include <map>

class Pipe : public rclcpp::Node {
private:
    rclcpp::CallbackGroup::SharedPtr _group_cloud;
    rclcpp::CallbackGroup::SharedPtr _group_info;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_cloud;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber_info;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_info;

    std::map<std::pair<int, int>, std::string> objects_types;
    sensor_msgs::msg::PointCloud2 _cloud;

    void _on_subscriber(sensor_msgs::msg::PointCloud2 cloud) {
        _cloud = cloud;
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
        _publisher_cloud->publish(msg);
    }

    std::map<std::pair<int, int>, std::string> parse_info(std_msgs::msg::String msg) {
        std::string data = msg.data;
        if (data == "{}")
            return objects_types;
        std::map<std::pair<int, int>, std::string> type_locations;
        data = data.substr(1,data.length()-2);// remove curly braces
        size_t num_objects = std::count(data.begin(), data.end(), ',') / 2 + 1;
        for (size_t i = 0; i < num_objects; i++) {
            std::pair<int, int> coords;
            int second_comma = data.find(',', data.find(',')+1);
            std::string object = data.substr(0, second_comma);
            data = data.substr(second_comma+1);
            // RCLCPP_INFO(get_logger(), "%s", object.c_str());
            // get ints between parens
            coords.first = std::stoi(object.substr(object.find('(')+1, object.find(',')));
            coords.second = std::stoi(object.substr(object.find(',')+1, object.find(')')));
            // RCLCPP_INFO(get_logger(), "FOUND COORDS: [%d, %d]", coords.first, coords.second);
            // get string in between '' 
            object = object.substr(object.find('\'') + 1, object.find('\''));
            type_locations[coords] = object;
        }
        // RCLCPP_INFO(get_logger(), "%ld, %s", num_objects, data.c_str());
        std_msgs::msg::String locations;
        locations.data = "";
        for (auto p : type_locations) {
            pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(_cloud, *cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conversion(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromPCLPointCloud2(*cloud, *cloud_conversion);
            // RCLCPP_INFO(get_logger(), ">%d, %d<  %ld", p.first.first, p.first.second, cloud_conversion->points.size());
            pcl::PointXYZ pt = cloud_conversion->points[p.first.first + p.first.second * cloud_conversion->width];
            locations.data += "[" + p.second + ": (" + std::to_string(pt.x) + " " + std::to_string(pt.y) +  + " " + std::to_string(pt.z) + ")]\n";
        }
        _publisher_info->publish(locations);
        return objects_types;
    }
    void _on_info(std_msgs::msg::String msg) {
        parse_info(msg);
    }

public:
    Pipe() : Node("pipe") {
        _group_cloud = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_cloud;
        options_cloud.callback_group = _group_cloud;
        _subscriber_cloud = create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/points", rclcpp::SensorDataQoS(), std::bind(&Pipe::_on_subscriber, this, std::placeholders::_1), options_cloud);
        _publisher_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("piped_pointcloud", 10);
        
        _group_info = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_info;
        options_info.callback_group = _group_info;
        _subscriber_info = create_subscription<std_msgs::msg::String>("detected_locations", 10, std::bind(&Pipe::_on_info, this, std::placeholders::_1), options_info);
        _publisher_info = create_publisher<std_msgs::msg::String>("space_to_type", 10);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto pipe = std::make_shared<Pipe>();
    rclcpp::executors::MultiThreadedExecutor ex;
    ex.add_node(pipe);
    ex.spin();
    rclcpp::shutdown();
    return 0;
}