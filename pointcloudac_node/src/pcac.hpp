#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

/*
AC for pointcloud updation
Check if the most recent pointcloud is different from the last pointcloud, and return a yes or no.
*/
using namespace BT;

class ac_pointcloud: public BT::AssumptionCheckerNode {

 private:
 auto pointcloud;
 auto prev;

 public:
 ac_pointcloud(const std::string& name, const NodeConfig& conf): BT::AssumptionCheckerNOde(name, conf) {}

 BT::NodeStatus tick() override {

    Expected<geometry_msgs::msg::PoseStamped> object_pose = getInput<geometry_msgs::msg::PoseStamped>("object_pose");

    if() {
     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%f]", object_pose.value().pose.position.z);
      return BT::NodeStatus::SUCCESS;
    }
    else{
      // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "NOT ON TABLE");
      return BT::NodeStatus::FAILURE;
    }
  }

};
class 