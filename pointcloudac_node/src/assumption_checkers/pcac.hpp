#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "behaviortree_ros2/bt_topic_checker_node.hpp"

/*
AC for pointcloud updation
Check if the most recent pointcloud is different from the last pointcloud, and return a yes or no.
*/
using namespace BT;

class ac_pointcloud: public RosTopicSubCheckerNode<sensor_msgs::msg::PointCloud2> {
 public:
 ac_pointcloud(
  const std::string& name, const NodeConfig& conf,  const RosNodeParams& params): 
  RosTopicSubCHeckerNode<sensor_msgs::msg::PointCloud2>(name, conf, params)
  {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({InputPort<std::string>("filtered_point_cloud")});
  }

  NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& last_msg) override {
    if(last_msg) {
      RCLCPP_INFO(logger(), "[%s] UPDATE!", name().c_str());
      return NodeStatus::SUCCESS;
    }
    else {
      
      RCLCPP_INFO(logger(), "NO UPDATE: %s", topic_name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

};
class 