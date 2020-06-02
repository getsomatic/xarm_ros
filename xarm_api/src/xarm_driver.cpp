/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: waylon <weile.wang@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm/linux/thread.h"
#include <unistd.h>

#define CMD_HEARTBEAT_US 3e7 // 30s

void* cmd_heart_beat(void* args)
{
    xarm_api::XARMDriver *my_driver = (xarm_api::XARMDriver *) args;
    while(true)
    {
        usleep(CMD_HEARTBEAT_US); // non-realtime
        my_driver->Heartbeat();
    }
    pthread_exit(0);
}

namespace xarm_api
{
    XARMDriver::~XARMDriver()
    {   
        // pthread_cancel(thread_id_);  // heartbeat related
        arm_cmd_->set_mode(XARM_MODE::POSE);
        arm_cmd_->close();
    }

    XARMDriver::XARMDriver() : Node("xarm_driver") {
        using std::placeholders::_1;
        using std::placeholders::_2;
        ///
        go_home_server_ = create_service<xarm_msgs::srv::Move>("go_home", std::bind(&XARMDriver::GoHomeCB, this, _1, _2));
        move_joint_server_ = create_service<xarm_msgs::srv::Move>("move_joint", std::bind(&XARMDriver::MoveJointCB, this, _1, _2));
        move_lineb_server_ = create_service<xarm_msgs::srv::Move>("move_lineb", std::bind(&XARMDriver::MoveLinebCB, this, _1, _2));
        move_line_server_ = create_service<xarm_msgs::srv::Move>("move_line", std::bind(&XARMDriver::MoveLineCB, this, _1, _2));
        move_servoj_server_ = create_service<xarm_msgs::srv::Move>("move_servoj", std::bind(&XARMDriver::MoveServoJCB, this, _1, _2));
        move_servo_cart_server_ = create_service<xarm_msgs::srv::Move>("move_servo_cart", std::bind(&XARMDriver::MoveServoCartCB, this, _1, _2));
        motion_ctrl_server_ = create_service<xarm_msgs::srv::SetAxis>("motion_ctrl", std::bind(&XARMDriver::MotionCtrlCB, this, _1, _2));
        set_mode_server_ = create_service<xarm_msgs::srv::SetInt16>("set_mode", std::bind(&XARMDriver::SetModeCB, this, _1, _2));
        set_state_server_ = create_service<xarm_msgs::srv::SetInt16>("set_state", std::bind(&XARMDriver::SetStateCB, this, _1, _2));
        set_tcp_offset_server_ = create_service<xarm_msgs::srv::TCPOffset>("set_tcp_offset", std::bind(&XARMDriver::SetTCPOffsetCB, this, _1, _2));
        set_load_server_ = create_service<xarm_msgs::srv::SetLoad>("set_load", std::bind(&XARMDriver::SetLoadCB, this, _1, _2));
        set_end_io_server_ = create_service<xarm_msgs::srv::SetDigitalIO>("set_digital_out", std::bind(&XARMDriver::SetDigitalIOCB, this, _1, _2));
        get_digital_in_server_ = create_service<xarm_msgs::srv::GetDigitalIO>("get_digital_in", std::bind(&XARMDriver::GetDigitalIOCB, this, _1, _2));
        get_analog_in_server_ = create_service<xarm_msgs::srv::GetAnalogIO>("get_analog_in", std::bind(&XARMDriver::GetAnalogIOCB, this, _1, _2));
        clear_err_server_ = create_service<xarm_msgs::srv::ClearErr>("clear_err", std::bind(&XARMDriver::ClearErrCB, this, _1, _2));
        gripper_config_server_ = create_service<xarm_msgs::srv::GripperConfig>("gripper_config", std::bind(&XARMDriver::GripperConfigCB, this, _1, _2));
        gripper_move_server_ = create_service<xarm_msgs::srv::GripperMove>("gripper_move", std::bind(&XARMDriver::GripperMoveCB, this, _1, _2));
        gripper_state_server_ = create_service<xarm_msgs::srv::GripperState>("gripper_state", std::bind(&XARMDriver::GripperStateCB, this, _1, _2));


        // state feedback topics:

        joint_state_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        robot_rt_state_ = create_publisher<xarm_msgs::msg::RobotMsg>("xarm_states", 10);
        end_input_state_ = create_publisher<xarm_msgs::msg::IOState>("xarm_input_states", 10);

        this->declare_parameter("dof");
         if (!this->get_parameter("dof", dof_)) {
             RCLCPP_ERROR(log_, "Failed to get parameter DOF. Shutting down...");
             assert(false);
         } else {
             RCLCPP_INFO(log_, "DOF set to %d", dof_);
         }
         // If you dont want to set DOF - you can use this line so it will be 7
        //this->get_parameter_or("portName", dof_, 7);
    }

    void XARMDriver::XARMDriverInit(char *server_ip)
    {
        arm_report_ = connext_tcp_report_norm(server_ip);
        // ReportDataNorm norm_data_;
        arm_cmd_ = connect_tcp_control(server_ip);
        if (arm_cmd_ == NULL)
            RCLCPP_ERROR(log_, "Xarm Connection Failed!");
        else // clear unimportant errors
        {
            // thread_id_ = thread_init(cmd_heart_beat, this); // heartbeat related
            int dbg_msg[16] = {0};
            arm_cmd_->servo_get_dbmsg(dbg_msg);

            for(int i=0; i<dof_; i++)
            {
                if((dbg_msg[i*2]==1)&&(dbg_msg[i*2+1]==40))
                {
                    arm_cmd_->clean_err();
                    RCLCPP_WARN(log_, "Cleared low-voltage error of joint %d", i+1);
                }
            }

        }
    }

    void XARMDriver::Heartbeat(void)
    {   
        int cmd_num;
        int ret = arm_cmd_->get_cmdnum(&cmd_num);
        // if(ret)
        // {
        //     ROS_ERROR("xArm Heartbeat error! ret = %d", ret);
        // }
        // ROS_INFO("xArm Heartbeat! %d", cmd_num);
    }

    bool XARMDriver::ClearErrCB(xarm_msgs::srv::ClearErr::Request::SharedPtr req, xarm_msgs::srv::ClearErr::Response::SharedPtr res)
    {
        // First clear controller warning and error:
        int ret1 = arm_cmd_->clean_war(); 
        int ret2 = arm_cmd_->clean_err();
        int ret3 = arm_cmd_->gripper_modbus_clean_err();
        // Then try to enable motor again:
        res->ret = arm_cmd_->motion_en(8, 1);

        if(res->ret)
        {
            res->message = "clear err, ret = "  + std::to_string(res->ret);
        }
        return true;

        // After calling this service, user should check '/xarm_states' again to make sure 'err' field is 0, to confirm success.
    }

    bool XARMDriver::MotionCtrlCB(xarm_msgs::srv::SetAxis::Request::SharedPtr req, xarm_msgs::srv::SetAxis::Response::SharedPtr res)
    {
        res->ret = arm_cmd_->motion_en(req->id, req->data);
        if(req->data == 1)
        {
            res->message = "motion enable, ret = "  + std::to_string(res->ret);
        }
        else
        {
            res->message = "motion disable, ret = " + std::to_string(res->ret);
        }
        return true;
    }

    bool XARMDriver::SetModeCB(xarm_msgs::srv::SetInt16::Request::SharedPtr req, xarm_msgs::srv::SetInt16::Response::SharedPtr res)
    {
        res->ret = arm_cmd_->set_mode(req->data);
        switch(req->data)
        {
            case XARM_MODE::POSE:
            {
               res->message = "pose mode, ret = " + std::to_string(res->ret);
            }break;
            case XARM_MODE::SERVO:
			{
				res->message = "servo mode, ret = " + std::to_string(res->ret);
			}break;
            case XARM_MODE::TEACH_CART:
			{
				res->message = "cartesian teach, ret = " + std::to_string(res->ret);
			} break;
            case XARM_MODE::TEACH_JOINT:
			{
				res->message = "joint teach , ret = " + std::to_string(res->ret);
			} break;
            default:
            {
                res->message = "the failed mode, ret = " + std::to_string(res->ret);
            }
        }

        return true;
    }

    bool XARMDriver::SetStateCB(xarm_msgs::srv::SetInt16::Request::SharedPtr req, xarm_msgs::srv::SetInt16::Response::SharedPtr res)
    {
        res->ret = arm_cmd_->set_state(req->data);
        switch(req->data)
        {
            case XARM_STATE::START:
            {
               res->message = "start, ret = " + std::to_string(res->ret);
            }break;
            case XARM_STATE::PAUSE:
            {
               res->message = "pause, ret = " + std::to_string(res->ret);
            }break;
            case XARM_STATE::STOP:
            {
               res->message = "stop, ret = " + std::to_string(res->ret);
            }break;
            default:
            {
                res->message = "the failed state, ret = " + std::to_string(res->ret);
            }
        }

        return true;
    }

    bool XARMDriver::SetTCPOffsetCB(xarm_msgs::srv::TCPOffset::Request::SharedPtr req, xarm_msgs::srv::TCPOffset::Response::SharedPtr res)
    {
        float offsets[6] = {req->x, req->y, req->z, req->roll, req->pitch, req->yaw};
        res->ret = arm_cmd_->set_tcp_offset(offsets);
        res->message = "set tcp offset: ret = " + std::to_string(res->ret); 
        return true;
    }

    bool XARMDriver::SetLoadCB(xarm_msgs::srv::SetLoad::Request::SharedPtr req, xarm_msgs::srv::SetLoad::Response::SharedPtr res)
    {   
        float Mass = req->mass;
        float CoM[3] = {req->xc, req->yc, req->zc};
        res->ret = arm_cmd_->set_tcp_load(Mass, CoM);
        res->message = "set load: ret = " + std::to_string(res->ret); 
        return true;
    }

    bool XARMDriver::SetDigitalIOCB(xarm_msgs::srv::SetDigitalIO::Request::SharedPtr req, xarm_msgs::srv::SetDigitalIO::Response::SharedPtr res)
    {
        res->ret = arm_cmd_->tgpio_set_digital(req->io_num, req->value);
        res->message = "set Digital port "+ std::to_string(req->io_num) +" to "+ std::to_string(req->value) + " : ret = " + std::to_string(res->ret); 
        return true;
    }

    bool XARMDriver::GetDigitalIOCB(xarm_msgs::srv::GetDigitalIO::Request::SharedPtr req, xarm_msgs::srv::GetDigitalIO::Response::SharedPtr res)
    {
        res->ret = arm_cmd_->tgpio_get_digital(&res->digital_1, &res->digital_2);
        res->message = "get Digital port ret = " + std::to_string(res->ret); 
        return true;
    }

    bool XARMDriver::GetAnalogIOCB(xarm_msgs::srv::GetAnalogIO::Request::SharedPtr req, xarm_msgs::srv::GetAnalogIO::Response::SharedPtr res)
    {
        switch (req->port_num)
        {
            case 1:
                res->ret = arm_cmd_->tgpio_get_analog1(&res->analog_value);
                break;
            case 2:
                res->ret = arm_cmd_->tgpio_get_analog2(&res->analog_value);
                break;

            default:
                res->message = "GetAnalogIO Fail: port number incorrect !";
                return false;
        }
        res->message = "get analog port ret = " + std::to_string(res->ret); 
        return true;
    }

    bool XARMDriver::GoHomeCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res)
    {
        res->ret = arm_cmd_->move_gohome(req->mvvelo, req->mvacc, req->mvtime);
        res->message = "go home, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveJointCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res)
    {
        float joint[1][7]={0};
        int index = 0;
        if(req->pose.size() != dof_)
        {
            res->ret = req->pose.size();
            res->message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return true;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                // joint[0][index] = req->pose[index];
                if(index<req->pose.size())
                    joint[0][index] = req->pose[index];
                else
                    joint[0][index] = 0;
            }
        }

        res->ret = arm_cmd_->move_joint(joint[0], req->mvvelo, req->mvacc, req->mvtime);
        res->message = "move joint, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveLineCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res)
    {
        float pose[1][6];
        int index = 0;
        if(req->pose.size() != 6)
        {
            res->ret = -1;
            res->message = "parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[0][index] = req->pose[index];
            }
        }

        res->ret = arm_cmd_->move_line(pose[0], req->mvvelo, req->mvacc, req->mvtime);
        res->message = "move line, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveLinebCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res)
    {
        float pose[1][6];
        int index = 0;
        if(req->pose.size() != 6)
        {
            res->ret = -1;
            res->message = "parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[0][index] = req->pose[index];
            }
        }

        res->ret = arm_cmd_->move_lineb(pose[0], req->mvvelo, req->mvacc, req->mvtime, req->mvradii);
        res->message = "move lineb, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveServoJCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res)
    {
        float pose[1][7]={0};
        int index = 0;
        if(req->pose.size() != dof_)
        {
            res->ret = req->pose.size();
            res->message = "pose parameters incorrect! Expected: "+std::to_string(dof_);
            return true;
        }
        else
        {
            for(index = 0; index < 7; index++) // should always send 7 joint commands, whatever current DOF is.
            {
                if(index<req->pose.size())
                    pose[0][index] = req->pose[index];
                else
                    pose[0][index] = 0;
            }
        }

        res->ret = arm_cmd_->move_servoj(pose[0], req->mvvelo, req->mvacc, req->mvtime);
        res->message = "move servoj, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveServoCartCB(xarm_msgs::srv::Move::Request::SharedPtr req, xarm_msgs::srv::Move::Response::SharedPtr res)
    {
        float pose[1][6];
        int index = 0;
        if(req->pose.size() != 6)
        {
            res->ret = -1;
            res->message = "MoveServoCartCB parameters incorrect!";
            return true;
        }
        else
        {
            for(index = 0; index < 6; index++)
            {
                pose[0][index] = req->pose[index];
            }
        }

        res->ret = arm_cmd_->move_servo_cartesian(pose[0], req->mvvelo, req->mvacc, req->mvtime);
        res->message = "move servo_cartesian, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GripperConfigCB(xarm_msgs::srv::GripperConfig::Request::SharedPtr req, xarm_msgs::srv::GripperConfig::Response::SharedPtr res)
    {
        if(req->pulse_vel>5000)
            req->pulse_vel = 5000;
        else if(req->pulse_vel<0)
            req->pulse_vel = 0;

        
        int ret1 = arm_cmd_->gripper_modbus_set_mode(0);
        int ret2 = arm_cmd_->gripper_modbus_set_en(1);
        int ret3 = arm_cmd_->gripper_modbus_set_posspd(req->pulse_vel);

        if(ret1 || ret2 || ret3)
        {
            res->ret = ret3;
        }
        else
        {
            res->ret = 0;
        }
        res->message = "gripper_config, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GripperMoveCB(xarm_msgs::srv::GripperMove::Request::SharedPtr req, xarm_msgs::srv::GripperMove::Response::SharedPtr res)
    {
        if(req->pulse_pos>850)
            req->pulse_pos = 850;
        else if(req->pulse_pos<-100)
            req->pulse_pos = -100;

        res->ret = arm_cmd_->gripper_modbus_set_pos(req->pulse_pos);
        res->message = "gripper_move, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GripperStateCB(xarm_msgs::srv::GripperState::Request::SharedPtr req, xarm_msgs::srv::GripperState::Response::SharedPtr res)
    {   
        int err_code = 0;
        float pos_now = 0;     
        if(arm_cmd_->gripper_modbus_get_errcode(&err_code))
            return false;

        if(arm_cmd_->gripper_modbus_get_pos(&pos_now))
            return false;
        
        res->err_code = err_code;
        res->curr_pos = pos_now;
        // fprintf(stderr, "gripper_pos: %f, gripper_err: %d\n", res->curr_pos, res->err_code);
        return true;
    }

    void XARMDriver::pub_robot_msg(xarm_msgs::msg::RobotMsg rm_msg)
    {
        robot_rt_state_->publish(rm_msg);
    }
    
    void XARMDriver::pub_joint_state(sensor_msgs::msg::JointState js_msg)
    {
        RCLCPP_ERROR(log_, "Joints J1=%lf, J2=%lf, J3=%lf, J4=%lf, J5=%lf, J6=%lf, J7=%lf", js_msg.position[0], js_msg.position[1], js_msg.position[2], js_msg.position[3], js_msg.position[4], js_msg.position[5], js_msg.position[6]);
        joint_state_->publish(js_msg);
    }

    void XARMDriver::pub_io_state()
    {
        arm_cmd_->tgpio_get_digital(&io_msg.digital_1, &io_msg.digital_2);
        arm_cmd_->tgpio_get_analog1(&io_msg.analog_1);
        arm_cmd_->tgpio_get_analog2(&io_msg.analog_2);

        end_input_state_->publish(io_msg);
    }

    int XARMDriver::get_frame(void)
    {
        int ret;
        ret = arm_report_->read_frame(rx_data_);
        return ret;
    }

    int XARMDriver::get_rich_data(ReportDataNorm &norm_data)
    {
        int ret;
        ret = norm_data_.flush_data(rx_data_);
        norm_data = norm_data_;
        return ret;
    }


}
