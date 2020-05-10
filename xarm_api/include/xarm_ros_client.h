#ifndef __XARM_ROS_CLIENT_H__
#define __XARM_ROS_CLIENT_H__

#include <rclcpp/rclcpp.hpp>
#include <xarm_driver.h>
#include <memory>
#include <chrono>
#include <functional>

namespace xarm_api{

class XArmROSClient : public rclcpp::Node
{
public:
	XArmROSClient();
	~XArmROSClient()= default;

	int motionEnable(short en);
	int setState(short state);
	int setMode(short mode);
	int setTCPOffset(const std::vector<float>& tcp_offset);
	int setLoad(float mass, const std::vector<float>& center_of_mass);
	int setServoJ(const std::vector<float>& joint_cmd);
	int setServoCartisian(const std::vector<float>& cart_cmd);
	int goHome(float jnt_vel_rad, float jnt_acc_rad=15);
	int moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad=15);
	int moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm=500);
	int moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm=500, float radii=0);

private:
	rclcpp::Client<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_client_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_client_;
	rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_state_client_;
  	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr go_home_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_lineb_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servoj_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_line_client_;
	rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_joint_client_;
	rclcpp::Client<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_client_;
	rclcpp::Client<xarm_msgs::srv::SetLoad>::SharedPtr set_load_client_;

    xarm_msgs::srv::SetAxis::Request::SharedPtr set_axis_srv_;
    xarm_msgs::srv::SetInt16::Request::SharedPtr set_int16_srv_;
    xarm_msgs::srv::TCPOffset::Request::SharedPtr offset_srv_;
    xarm_msgs::srv::SetLoad::Request::SharedPtr set_load_srv_;
    xarm_msgs::srv::Move::Request::SharedPtr move_srv_;
    xarm_msgs::srv::Move::Request::SharedPtr servoj_msg_;
    xarm_msgs::srv::Move::Request::SharedPtr servo_cart_msg_;

    rclcpp::Logger log_ = rclcpp::get_logger("XArmROSClient");
};

}

#endif
