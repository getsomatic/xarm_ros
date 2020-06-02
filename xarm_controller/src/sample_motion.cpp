/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#define PUB_RATE 100.0
#define ACC_TIME 0.5
#define CONST_V_TIME 3.0
#define DOF 7

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    // "angle_control"

    auto node = std::make_shared<rclcpp::Node>("angle_control");
   
    auto angle1_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint1_position_controller/command", 100);
    auto angle2_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint2_position_controller/command", 100);
    auto angle3_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint3_position_controller/command", 100);
    auto angle4_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint4_position_controller/command", 100);
    auto angle5_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint5_position_controller/command", 100);
    auto angle6_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint6_position_controller/command", 100);
    auto angle7_pub = node->create_publisher<std_msgs::msg::Float64>("/xarm/joint7_position_controller/command", 100);

    rclcpp::executors::SingleThreadedExecutor singleThreadedExecutor;
    singleThreadedExecutor.add_node(node);

    rclcpp::Rate loop_rate(PUB_RATE);

    unsigned int i = 0;
    int dir = -1;
    double angle_cmd = 0.0;


    std_msgs::msg::Float64 msg; /* Float64 is a struct defined in std_msgs */

    double delta_s = 0.000628 / (PUB_RATE / 100.0) * 4;
    double const_v = delta_s * PUB_RATE;
    double acc = const_v / ACC_TIME;
    double acc_t = 0.0;
    double cmd_before_dec = 0.0;



    int robot_dof = DOF;

    while (rclcpp::ok())
    {

        if (i <= ACC_TIME * PUB_RATE /*&& dir == -1*/)
        {
            acc_t = 1.0 / PUB_RATE * i;
            angle_cmd = - 0.5 * (acc) * (acc_t) * (acc_t);
        }

        else if (i <= (ACC_TIME + CONST_V_TIME)*PUB_RATE)
        {
            angle_cmd = angle_cmd + dir * delta_s;
            if (i == (ACC_TIME + CONST_V_TIME)*PUB_RATE)
                cmd_before_dec = angle_cmd;
        }

        else
        {
            acc_t = 1.0 / PUB_RATE * (i - (ACC_TIME + CONST_V_TIME) * PUB_RATE);
            angle_cmd = cmd_before_dec - const_v * acc_t + 0.5 * (acc) * (acc_t) * (acc_t);
        }


        i = i - dir;

        if (i == (2 * ACC_TIME + CONST_V_TIME)*PUB_RATE + 1)
        {
            dir = 1;
        }
        else if (i == 0)
        {
            dir = -1;
        }


        msg.data = angle_cmd;


        // Give J2 and J4 same angle_cmd from calculation
        msg.data = -msg.data;
        angle4_pub->publish(msg);
        msg.data = -msg.data;

        angle2_pub->publish(msg);

        // Give J1 and J5 twice as angle_cmd from calculation
        msg.data = msg.data * 2;

        angle1_pub->publish(msg);

        angle5_pub->publish(msg);

        // Other joint command fixed to zero
        msg.data = 0.0;

        angle3_pub->publish(msg);

        angle6_pub->publish(msg);

        angle7_pub->publish(msg);

        break;


        singleThreadedExecutor.spin_once();
        loop_rate.sleep();
    }

    fprintf(stderr, "ROS::OK() false \n" );
    return 0;
}
