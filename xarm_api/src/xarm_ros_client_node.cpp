//
// Created by max on 11.02.21.
//

#include <rclcpp/rclcpp.hpp>
#include <xarm_ros_client.h>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("XArmROSClient");
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr client =
            node->create_client<xarm_msgs::srv::Move>("move_joint");

    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvacc = 15;

    request->mvtime = 0;
    request->mvvelo = 0.5;

    request->pose = {0 * M_PI / 180,15 * M_PI / 180 * M_PI / 180,-180 * M_PI / 180,180 * M_PI / 180,0 * M_PI / 180,75 * M_PI / 180,0 * M_PI / 180};

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ret);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service move_joint");
        return 1;
    }

    return 0;
}