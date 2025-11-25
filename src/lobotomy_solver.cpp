#include <functional>
#include <memory>
#include <iostream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

bool send_move(rclcpp::Node::SharedPtr node,
               rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client,
               const std::string &dir)
{
    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;

    auto future = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call /move_command");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Move '%s' executed.", dir.c_str());
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("lobotomy_client");
    auto move_client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    if (!move_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service /move_command not available.");
        return 1;
    }

    while (1) {
        // xd
        int random = rand() % 4;
        if (random == 0) {send_move(node, move_client, "up");}
        else if (random == 1) {send_move(node, move_client, "down");}
        else if (random == 2) {send_move(node, move_client, "left");}
        else if (random == 3) {send_move(node, move_client, "right");}
    }

    rclcpp::shutdown();
    return 0;
}