/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#include "xarm_hw.h"
#include "string"
namespace xarm_control
{

    XArmHW::XArmHW(rclcpp::Node::SharedPtr node): node_(node) {

    }

	void XArmHW::clientInit(const std::string& robot_ip)
	{
		position_cmd_.resize(dof_);
		position_cmd_float_.resize(dof_); // command vector must have 7 dimention!
		position_fdb_.resize(dof_);
		velocity_cmd_.resize(dof_);
		velocity_fdb_.resize(dof_);
		effort_cmd_.resize(dof_);
		effort_fdb_.resize(dof_);

		curr_err = 0;
		curr_state = 0;

		pos_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(jnt_state_topic, 100, std::bind(&XArmHW::pos_fb_cb, this, std::placeholders::_1));
        state_sub_ = node_->create_subscription<xarm_msgs::msg::RobotMsg>(xarm_state_topic, 100, std::bind(&XArmHW::state_fb_cb, this, std::placeholders::_1));

		for(unsigned int j=0; j < dof_; j++)
	  	{
	  		// Create joint state interface for all joints
	    	js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_fdb_[j], &velocity_fdb_[j], &effort_fdb_[j]));


	    	/*hardware_interface::JointHandle joint_handle;
	    	joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]),&position_cmd_[j]);
	    	*/
	    	// TODO: Think which of them to choose
	    	hardware_interface::JointCommandHandle joint_handle1;
	    	joint_handle1 = hardware_interface::JointCommandHandle(js_interface_.getHandle(jnt_names_[j]),&position_cmd_[j]);

	    	hardware_interface::JointStateHandle joint_handle2;
	    	joint_handle2 = hardware_interface::JointStateHandle(js_interface_.getHandle(jnt_names_[j]),&position_cmd_[j]);
	    	//pj_interface_.registerHandle(joint_handle);
	  	}

	  	registerInterface(&js_interface_);
	  	registerInterface(&pj_interface_);
	  	
	  	int ret1 = xarm->motionEnable(1);
	  	int ret2 = xarm->setMode(XARM_MODE::SERVO);
	  	int ret3 = xarm->setState(XARM_STATE::START);

	  	if(ret3)
	  	{
	  		RCLCPP_ERROR(rclcpp::get_logger("XArmHW"),"The Xarm may not be properly connected or hardware error exists, PLEASE CHECK or RESTART HARDWARE!!!");
            RCLCPP_ERROR(rclcpp::get_logger("XArmHW")," ");
            RCLCPP_ERROR(rclcpp::get_logger("XArmHW"),"Did you specify the correct ros param xarm_robot_ip ? Exitting...");
	  		rclcpp::shutdown();
	  		exit(1);
	  	}

	}

	int XArmHW::init()
	{
        std::string hw_ns(node_->get_namespace());
        hw_ns+="/";
        

		ros::service::waitForService(hw_ns+"motion_ctrl");
	  	ros::service::waitForService(hw_ns+"set_state");
	  	ros::service::waitForService(hw_ns+"set_mode");
	  	ros::service::waitForService(hw_ns+"move_servoj");
		xarm = std::make_shared<xarm_api::XArmROSClient>();
		std::string robot_ip;
		std::vector<std::string> jnt_names;
		int xarm_dof = 0;

		if(!robot_hw_nh.hasParam("DOF"))
		{
            RCLCPP_ERROR(rclcpp::get_logger("XArmHW"),"ROS Parameter xarm_dof not specified!");
			return false;
		}
		if(!robot_hw_nh.hasParam("xarm_robot_ip"))
		{
            RCLCPP_ERROR(rclcpp::get_logger("XArmHW"),"ROS Parameter xarm_robot_ip not specified!");
			return false;
		}

		/* getParam forbids to change member */
		robot_hw_nh.getParam("DOF", xarm_dof);
		robot_hw_nh.getParam("xarm_robot_ip", robot_ip);
		robot_hw_nh.getParam("joint_names", jnt_names);
		dof_ = xarm_dof;
		jnt_names_ = jnt_names;

		clientInit(robot_ip);
		return true;
	}

	XArmHW::~XArmHW()
	{
		xarm->setMode(XARM_MODE::POSE);
	}

	void XArmHW::pos_fb_cb(sensor_msgs::msg::JointState::SharedPtr data)
	{
		for(int j=0; j<dof_; j++)
		{
			position_fdb_[j] = data->position[j];
			velocity_fdb_[j] = data->velocity[j];
			effort_fdb_[j] = data->effort[j];
		}
	}

	void XArmHW::state_fb_cb(xarm_msgs::msg::RobotMsg::SharedPtr data)
	{
		curr_mode = data->mode;
		curr_state = data->state;
		curr_err = data->err;
	}

	void XArmHW::read(const rclcpp::Time& time, const rclcpp::Duration& period)
	{
		// basically the above feedback callback functions have done the job
	}

	void XArmHW::write(const rclcpp::Time& time, const rclcpp::Duration& period)
	{

		for(int k=0; k<dof_; k++)
		{
			position_cmd_float_[k] = (float)position_cmd_[k];
		}

		xarm->setServoJ(position_cmd_float_);

	}

	void XArmHW::get_status(int state_mode_err[3])
	{
		state_mode_err[0] = curr_state;
		state_mode_err[1] = curr_mode;
		state_mode_err[2] = curr_err;
	}

	bool XArmHW::need_reset()
	{
		if(curr_state==4 || curr_state==5 || curr_err)
			return true;
		else
			return false;
	}
}

PLUGINLIB_EXPORT_CLASS(xarm_control::XArmHW, hardware_interface::RobotHardware)