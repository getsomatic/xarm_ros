/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: waylon <weile.wang@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include <xarm/linux/thread.h>
#include "xarm/connect.h"
#include "xarm/report_data.h"
#include <memory>
#include <chrono>
#include <functional>

class XarmRTConnection : public rclcpp::Node
{
    public:
        XarmRTConnection(std::shared_ptr<xarm_api::XARMDriver> drv) : Node("XarmRTConnection")
        {
            char server_ip[20]={0};
            std::string robot_ip = "192.168.1.121";

            this->declare_parameter("robot_ip");
            if (!this->get_parameter("robot_ip", robot_ip)) {
                RCLCPP_ERROR(log_, "Failed to get parameter xarm_robot_ip. Shutting down...");
                assert(false);
            } else {
                RCLCPP_INFO(log_, "Got IP %s", robot_ip.c_str());
            }

            strcpy(server_ip, robot_ip.c_str());

            this->declare_parameter("dof");
            if (!this->get_parameter("dof", joint_num_)) {
                RCLCPP_ERROR(log_, "Failed to get parameter DOF. Shutting down...");
                assert(false);
            } else {
                RCLCPP_INFO(log_, "DOF set to %d", joint_num_);
            }

            this->declare_parameter("joint_names");
            if (!this->get_parameter("joint_names", joint_name_)) {
                RCLCPP_ERROR(log_, "Failed to get parameter joint_names. Shutting down...");
                assert(false);
            } else {
                RCLCPP_INFO(log_, "Got joint names successfully");
            }
            ip = server_ip;
            xarm_driver = drv;
            xarm_driver->XARMDriverInit(server_ip);
            thread_id = thread_init(thread_proc, (void *)this);

        }

    [[noreturn]] void thread_run(void)
        {
            int ret;
            int err_num;
            int rxcnt;
            int i;
            int first_cycle = 1;
            double d, prev_angle[joint_num_];

            rclcpp::Rate r(REPORT_RATE_HZ); // 50Hz

            while(true)
            {
                // usleep(5000);
                ret = xarm_driver->get_frame();
                if (ret != 0) continue;

                ret = xarm_driver->get_rich_data(norm_data);
                if (ret == 0)
                {
                    rxcnt++;

                    now = clock_.now();
                    js_msg.header.stamp = now;
                    js_msg.header.frame_id = "real-time data";
                    js_msg.name.resize(joint_num_);
                    js_msg.position.resize(joint_num_);
                    js_msg.velocity.resize(joint_num_);
                    js_msg.effort.resize(joint_num_);
                    for(i = 0; i < joint_num_; i++)
                    {
                        d = (double)norm_data.angle_[i];
                        js_msg.name[i] = joint_name_[i];
                        js_msg.position[i] = d;

                        if (first_cycle)
                        {
                            js_msg.velocity[i] = 0;
                            first_cycle = 0;
                        }
                        else
                        {
                            js_msg.velocity[i] = (js_msg.position[i] - prev_angle[i])*REPORT_RATE_HZ;
                        }

                        js_msg.effort[i] = 0;

                        prev_angle[i] = d;
                    }
                    
                    xarm_driver->pub_joint_state(js_msg);

                    rm_msg.state = norm_data.runing_;
                    rm_msg.mode = norm_data.mode_;
                    rm_msg.cmdnum = norm_data.cmdnum_;
                    rm_msg.err = norm_data.err_;
                    rm_msg.warn = norm_data.war_;
                    rm_msg.mt_brake = norm_data.mt_brake_;
                    rm_msg.mt_able = norm_data.mt_able_;
                    rm_msg.angle.resize(joint_num_);
                    
                    for(i = 0; i < joint_num_; i++)    
                    {
                        /* set the float precision*/
                        double d = norm_data.angle_[i];
                        double r;
                        char str[8];
                        sprintf(str, "%0.3f", d); 
                        sscanf(str, "%lf", &r);
                        rm_msg.angle[i] = r;
                    }
                    for(i = 0; i < 6; i++)    
                    {
                        rm_msg.pose[i] = norm_data.pose_[i];
                        rm_msg.offset[i] = norm_data.tcp_offset_[i];
                    }
                    xarm_driver->pub_robot_msg(rm_msg);

                    // publish io state: This line may interfere with servoj execution
                    // xarm_driver.pub_io_state();

                }
                else 
                {
                    printf("Error: real_data.flush_data failed, ret = %d\n", ret);
                    err_num++;
                }

                r.sleep();
            }
        }

        static void* thread_proc(void *arg) 
        {
            XarmRTConnection* pThreadTest=(XarmRTConnection*)arg;
            pThreadTest->thread_run();
        }

    public:
        pthread_t thread_id;
        char *ip;
        rclcpp::Time now;
        SocketPort *arm_report;
        ReportDataNorm norm_data;
        sensor_msgs::msg::JointState js_msg;
        std::shared_ptr<xarm_api::XARMDriver> xarm_driver;
        xarm_msgs::msg::RobotMsg rm_msg;

        int joint_num_;
        std::vector<std::string> joint_name_;
        constexpr static const double REPORT_RATE_HZ = 10; /* 10Hz, same with norm_report frequency */
        rclcpp::Clock clock_;
        rclcpp::Logger log_ = rclcpp::get_logger("XarmRTConnection");
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto xarm_driver = std::make_shared<xarm_api::XARMDriver>();
    RCLCPP_INFO(rclcpp::get_logger("xarm_node") ,"Starting XARM driver.");

    XarmRTConnection rt_connect(xarm_driver);

    rclcpp::spin(xarm_driver);
    rclcpp::shutdown();
    return 0;
}
