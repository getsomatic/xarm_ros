/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <rclcpp/rclcpp.hpp>
#include <combined_robot_hw/combined_robot_hw.h>
#include "xarm_hw.h"

int main(int argc, char**argv)
{
	rclcpp::init(argc, argv); //, "xarm_controller");
    rclcpp::Rate r(100);

    rclcpp::Rate  r1(0.5);
    r1.sleep();
    auto multiThreadedExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<rclcpp::Node>("xarm_control_node");
    multiThreadedExecutor->add_node(node);

    auto xarm_hw = std::make_shared<xarm_control::XArmHW>(node);
	if(!xarm_hw->init()) exit(-1);

	controller_manager::ControllerManager cm(xarm_hw, multiThreadedExecutor);

    multiThreadedExecutor->spin();

	// IMPORTANT: DO NOT REMOVE THIS DELAY !!!
	/* Wait for correct initial position to be updated to ros_controller */
    r1.sleep();
    rclcpp::Clock clock;
    rclcpp::Time ts = clock.now();
	while (rclcpp::ok())
	{
        rclcpp::Duration elapsed = clock.now() - ts;
	   ts = clock.now();
	   // xarm_hw.read(ts, elapsed);
	   cm.update(ts, elapsed, xarm_hw.need_reset()); // reset_controllers=true: preempt and cancel current goal
	   
	   xarm_hw->write(ts, elapsed);
	   r.sleep();
	}

	rclcpp::shutdown();
	return 0;
}