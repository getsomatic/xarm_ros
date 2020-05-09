#ifndef __XARM_DRIVER_H
#define __XARM_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/tcp_offset.hpp>
#include <xarm_msgs/srv/set_load.hpp>
#include <xarm_msgs/srv/set_axis.hpp>
#include <xarm_msgs/srv/move.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_msgs/msg/io_state.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>
#include <xarm_msgs/srv/get_digital_io.hpp>
#include <xarm_msgs/srv/get_analog_io.hpp>
#include <xarm_msgs/srv/clear_err.hpp>
#include <xarm_msgs/srv/gripper_config.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/gripper_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm/common/data_type.h>
#include <xarm/linux/thread.h>
#include "xarm/connect.h"
#include "xarm/report_data.h"

namespace xarm_api
{
class XARMDriver : public rclcpp::Node
    {
        public:
            XARMDriver();;
            ~XARMDriver();
            void XARMDriverInit(char *server_ip);
            void Heartbeat(void);
            // provide a list of services:
            bool MotionCtrlCB(xarm_msgs::srv::SetAxis::Request::SharedPtr req, xarm_msgs::srv::SetAxis::Response::SharedPtr res);
            bool SetModeCB(xarm_msgs::srv::SetInt16::Request::SharedPtr req, xarm_msgs::srv::SetInt16::Response::SharedPtr res);
            bool SetStateCB(xarm_msgs::srv::SetInt16::Request::SharedPtr req, xarm_msgs::srv::SetInt16::Response::SharedPtr res);
            bool SetTCPOffsetCB(xarm_msgs::srv::TCPOffset::Request::SharedPtr req, xarm_msgs::srv::TCPOffset::Response::SharedPtr res);
            bool SetLoadCB(xarm_msgs::srv::SetLoad::Request::SharedPtr req, xarm_msgs::srv::SetLoad::Response::SharedPtr res);
            bool SetDigitalIOCB(xarm_msgs::srv::SetDigitalIO::Request::SharedPtr req, xarm_msgs::srv::SetDigitalIO::Response::SharedPtr res);
            bool GetDigitalIOCB(xarm_msgs::srv::GetDigitalIO::Request::SharedPtr req, xarm_msgs::srv::GetDigitalIO::Response::SharedPtr res);
            bool GetAnalogIOCB(xarm_msgs::srv::GetAnalogIO::Request::SharedPtr req, xarm_msgs::srv::GetAnalogIO::Response::SharedPtr res);
            bool ClearErrCB(xarm_msgs::srv::ClearErr::Request::SharedPtr req, xarm_msgs::srv::ClearErr::Response::SharedPtr res);
            bool GoHomeCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res);
            bool MoveJointCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res);
            bool MoveLinebCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res);
            bool MoveLineCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res);
            bool MoveServoJCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res);
            bool MoveServoCartCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res);
            bool GripperConfigCB(xarm_msgs::srv::GripperConfig::Request::SharedPtr req, xarm_msgs::srv::GripperConfig::Response::SharedPtr res);
            bool GripperMoveCB(xarm_msgs::srv::GripperMove::Request::SharedPtr req, xarm_msgs::srv::GripperMove::Response::SharedPtr res);
            bool GripperStateCB(xarm_msgs::srv::GripperState::Request::SharedPtr req, xarm_msgs::srv::GripperState::Response::SharedPtr res);

            void pub_robot_msg(xarm_msgs::msg::RobotMsg rm_msg);
            void pub_joint_state(sensor_msgs::msg::JointState js_msg);
            void pub_io_state();

            int get_frame(void);
            int get_rich_data(ReportDataNorm &norm_data);

        private:
            SocketPort *arm_report_;
            ReportDataNorm norm_data_;
            UxbusCmd *arm_cmd_;
            unsigned char rx_data_[1280];
            std::string ip;
            pthread_t thread_id_;
            int dof_;
            xarm_msgs::msg::IOState io_msg;

            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr go_home_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_joint_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_lineb_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servoj_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_server_;
            rclcpp::Service<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_server_;
            rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_state_server_;
            rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_server_;
            rclcpp::Service<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_server_;
            rclcpp::Service<xarm_msgs::srv::SetLoad>::SharedPtr set_load_server_;
            rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_end_io_server_;
            rclcpp::Service<xarm_msgs::srv::GetDigitalIO>::SharedPtr get_digital_in_server_;
            rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_analog_in_server_;
            rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_server_;
            rclcpp::Service<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_server_;
            rclcpp::Service<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_server_;
            rclcpp::Service<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_server_;

            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
            rclcpp::Publisher<xarm_msgs::msg::RobotMsg>::SharedPtr robot_rt_state_;

            // For some reason it is commented in XARM ros1 driver initilisation, so I will left it here as is.
            rclcpp::Publisher<xarm_msgs::msg::IOState>::SharedPtr end_input_state_;

            rclcpp::Logger log_ = rclcpp::get_logger("XARMDriver");
    };
}

#endif
