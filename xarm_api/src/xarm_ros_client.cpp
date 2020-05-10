/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_ros_client.h>

namespace xarm_api{


XArmROSClient::XArmROSClient() : Node("XArmROSClient") {
    /// Initialising srv Messages
    set_axis_srv_ = std::make_shared<xarm_msgs::srv::SetAxis::Request>();
    set_int16_srv_ = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    offset_srv_ = std::make_shared<xarm_msgs::srv::TCPOffset::Request>();
    set_load_srv_ = std::make_shared<xarm_msgs::srv::SetLoad::Request>();
    move_srv_ = std::make_shared<xarm_msgs::srv::Move::Request>();
    servoj_msg_ = std::make_shared<xarm_msgs::srv::Move::Request>();
    servo_cart_msg_ = std::make_shared<xarm_msgs::srv::Move::Request>();


    /// Initialising Service Clients
    motion_ctrl_client_ = create_client<xarm_msgs::srv::SetAxis>("motion_ctrl");
    set_mode_client_ = create_client<xarm_msgs::srv::SetInt16>("set_mode");
    set_state_client_ = create_client<xarm_msgs::srv::SetInt16>("set_state");
    set_tcp_offset_client_ = create_client<xarm_msgs::srv::TCPOffset>("set_tcp_offset");
    set_load_client_ = create_client<xarm_msgs::srv::SetLoad>("set_load");
    go_home_client_ = create_client<xarm_msgs::srv::Move>("go_home");
    move_lineb_client_ = create_client<xarm_msgs::srv::Move>("move_lineb");
    move_line_client_ = create_client<xarm_msgs::srv::Move>("move_line");
    move_joint_client_ = create_client<xarm_msgs::srv::Move>("move_joint");
    rmw_qos_profile_t reliable_profile;
    reliable_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    move_servoj_client_ = create_client<xarm_msgs::srv::Move>("move_servoj", reliable_profile); // persistent connection for servoj
    move_servo_cart_client_ = create_client<xarm_msgs::srv::Move>("move_servo_cart", reliable_profile); // persistent connection for servo_cartesian
}

int XArmROSClient::motionEnable(short en)
{
    set_axis_srv_->id = 8;
    set_axis_srv_->data = en;

    auto result = motion_ctrl_client_->async_send_request(set_axis_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(log_, "%s\n", result.get()->message.c_str());
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service motion_ctrl");
        return 1;
    }
}

int XArmROSClient::setState(short state)
{
	set_int16_srv_->data = state;
    auto result = set_state_client_->async_send_request(set_int16_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(log_, "%s\n", result.get()->message.c_str());
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service set_state");
        return 1;
    }
}

int XArmROSClient::setMode(short mode)
{
	set_int16_srv_->data = mode;

    auto result = set_mode_client_->async_send_request(set_int16_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(log_, "%s\n", result.get()->message.c_str());
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service set_mode");
        return 1;
    }
}

int XArmROSClient::setServoJ(const std::vector<float>& joint_cmd)
{
	servoj_msg_->mvvelo = 0;
    servoj_msg_->mvacc = 0;
    servoj_msg_->mvtime = 0;
    servoj_msg_->pose = joint_cmd;

    auto result = move_servoj_client_->async_send_request(servoj_msg_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        //RCLCPP_INFO(log_, "%s\n", result.get()->message.c_str());
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service move_servoj");
        return 1;
    }
}

int XArmROSClient::setServoCartisian(const std::vector<float>& cart_cmd)
{
    servo_cart_msg_->mvvelo = 0;
    servo_cart_msg_->mvacc = 0;
    servo_cart_msg_->mvtime = 0;
    servo_cart_msg_->pose = cart_cmd;

    auto result = move_servo_cart_client_->async_send_request(servo_cart_msg_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        //RCLCPP_INFO(log_, "%s\n", result.get()->message.c_str());
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service move_servo_cart");
        return 1;
    }
}

int XArmROSClient::setTCPOffset(const std::vector<float>& tcp_offset)
{
    if(tcp_offset.size() != 6)
    {
        RCLCPP_ERROR(log_, "Set tcp offset service parameter should be 6-element Cartesian offset!");
        return 1;
    }
    
    offset_srv_->x = tcp_offset[0];
    offset_srv_->y = tcp_offset[1];
    offset_srv_->z = tcp_offset[2];
    offset_srv_->roll = tcp_offset[3];
    offset_srv_->pitch = tcp_offset[4];
    offset_srv_->yaw = tcp_offset[5];

    auto result = set_tcp_offset_client_->async_send_request(offset_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service set_tcp_offset");
        return 1;
    }
}

int XArmROSClient::setLoad(float mass, const std::vector<float>& center_of_mass)
{
    set_load_srv_->mass = mass;
    set_load_srv_->xc = center_of_mass[0];
    set_load_srv_->yc = center_of_mass[1];
    set_load_srv_->zc = center_of_mass[2];

    auto result = set_load_client_->async_send_request(set_load_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service set_load");
        return 1;
    }
}

int XArmROSClient::goHome(float jnt_vel_rad, float jnt_acc_rad)
{
    move_srv_->mvvelo = jnt_vel_rad;
    move_srv_->mvacc = jnt_acc_rad;
    move_srv_->mvtime = 0;

    auto result = go_home_client_->async_send_request(move_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service go_home");
        return 1;
    }
}

int XArmROSClient::moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad)
{
    move_srv_->mvvelo = jnt_vel_rad;
    move_srv_->mvacc = jnt_acc_rad;
    move_srv_->mvtime = 0;
    move_srv_->pose = joint_cmd;

    auto result = move_joint_client_->async_send_request(move_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service move_joint");
        return 1;
    }
}

int XArmROSClient::moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm)
{
    move_srv_->mvvelo = cart_vel_mm;
    move_srv_->mvacc = cart_acc_mm;
    move_srv_->mvtime = 0;
    move_srv_->pose = cart_cmd;

    auto result = move_joint_client_->async_send_request(move_srv_);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get()->ret;
    } else {
        RCLCPP_ERROR(log_, "Failed to call service move_line");
        return 1;
    }
}

int XArmROSClient::moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm, float radii)
{
    move_srv_->mvvelo = cart_vel_mm;
    move_srv_->mvacc = cart_acc_mm;
    move_srv_->mvtime = 0;
    move_srv_->mvradii = radii;
    
    for(int i=0; i<num_of_pnts; i++)
    {
        move_srv_->pose = cart_cmds[i];

        auto result = move_joint_client_->async_send_request(move_srv_);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            return result.get()->ret;
        } else {
            RCLCPP_ERROR(log_, "Failed to call service move_lineb");
            return 1;
        }
    }
    return 0;
}

}
