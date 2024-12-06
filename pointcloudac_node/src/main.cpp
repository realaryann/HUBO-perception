#include <filesystem>
#include "assumption_checkers/pcac.hpp"

BT::NodeStatus Wait() {
    rclcpp::sleep_for(std::chrono::seconds(3));
    return BT::NodeStatus::SUCCESS;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("test_bt");
    nh->declare_parameter("using_robot", false);

    rclcpp::Parameter using_robot_param = nh->get_parameter("using_robot");
    bool using_robot = using_robot_param.as_bool();

    // Create BT Nodes
    BehaviorTreeFactory factory;
    RosNodeParams pars;
    pars.nh = nh;
    pars.qos_profile=rclcpp::SensorDataQoS();
    pars.qos_profile=rclcpp::SystemDefaultsQoS();
    pars.server_timeout = std::chrono::milliseconds(20000);
    factory.registerSimpleAction("Wait", [&](TreeNode&) {return Wait();});
    factory.registerNodeType<PointCloudAC>("PointCloudAC", pars);
    return 0;
}