#pragma once

#include <ros/ros.h>
#include "arx_r5_src/interfaces/InterfacesThread.hpp"
#include "arx_r5_msg/RobotCmd.h"
#include "arx_r5_msg/RobotStatus.h"
#include <chrono>
#include <memory>

namespace arx::r5
{
    class R5Controller
    {
    public:
        R5Controller(ros::NodeHandle nh);

        void CmdCallback(const arx_r5_msg::RobotCmd::ConstPtr& msg);
        void PubState(const ros::TimerEvent&);

    private:
        std::shared_ptr<InterfacesThread> r5_Interfaces_ptr_;

        ros::Publisher joint_state_publisher_;
        ros::Subscriber joint_state_subscriber_;
        ros::Timer timer_;
    };
}
