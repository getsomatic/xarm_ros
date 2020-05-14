/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#ifndef __XARM_HARDWARE_INTERFACE_H__
#define __XARM_HARDWARE_INTERFACE_H__

// ros_control
#include <control_toolbox/pid.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/robot_hardware_interface.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// for mutex
#include <pthread.h>
// xarm
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm_ros_client.h"
#include "xarm_msgs/msg/robot_msg.hpp"

namespace xarm_control
{
	const std::string jnt_state_topic = "joint_states";
	const std::string xarm_state_topic = "xarm_states";

	class XArmHW : public hardware_interface::RobotHardware
	{
	public:
		XArmHW();
		~XArmHW() override;
		virtual int init();
		virtual void read(const rclcpp::Time& time, const rclcpp::Duration& period);
		virtual void write(const rclcpp::Time& time, const rclcpp::Duration& period);

		std::shared_ptr<xarm_api::XArmROSClient> XArmROSClient();

		/* TODO:
		virtual bool prepareSwitch(const std::list<ControllerInfo>& start_list,
	                             const std::list<ControllerInfo>& stop_list) { return true; }
	  	virtual void doSwitch(const std::list<ControllerInfo>& ,
	                        const std::list<ControllerInfo>& ) {}*/
		
		/* get current arm status: in the order of state, mode and error_code */
		void get_status(int state_mode_err[3]);
		/* check whether the controller needs to be reset due to error or mode change */
		bool need_reset();

        hardware_interface::hardware_interface_ret_t read() override;

        hardware_interface::hardware_interface_ret_t write() override;

    private:
		int curr_state;
		int curr_mode;
		int curr_err;

		unsigned int dof_;
		std::vector<std::string> jnt_names_;
		std::vector<double> position_cmd_;
		std::vector<float> position_cmd_float_;
		std::vector<double> velocity_cmd_;
		std::vector<double> effort_cmd_;

		std::vector<double> position_fdb_;
		std::vector<double> velocity_fdb_;
		std::vector<double> effort_fdb_;

        std::shared_ptr<xarm_api::XArmROSClient> xarm;
		rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pos_sub_;
        rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr state_sub_;

		void clientInit(const std::string& robot_ip);
		void pos_fb_cb(sensor_msgs::msg::JointState::SharedPtr data);
		void state_fb_cb(xarm_msgs::msg::RobotMsg::SharedPtr data);
	};

}

#endif
