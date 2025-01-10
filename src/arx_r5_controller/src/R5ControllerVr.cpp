#include "arx_r5_controller/R5ControllerVr.hpp"

// using namespace std::chrono_literals;

namespace arx::r5
{
    R5Controller::R5Controller(ros::NodeHandle nh)
    {
        // 创建发布器
        joint_state_publisher_ = nh.advertise<arx_msgs::PosCmd>("/r5_status", 10);
        // 创建订阅器
        joint_state_subscriber_ = nh.subscribe<arx_msgs::PosCmd>(
            "/ARX_VR_L", 10, &R5Controller::CmdCallback, this);
        // 定时器，用于发布关节信息

        timer_ = nh.createTimer(ros::Duration(0.01), &R5Controller::PubState, this);
        r5_Interfaces_ptr_ = std::make_shared<InterfacesThread>("can0",0);
    }

    void R5Controller::CmdCallback(const arx_msgs::PosCmd::ConstPtr& msg)
    {
        double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

        r5_Interfaces_ptr_->setEndPose(transform);

        r5_Interfaces_ptr_->setArmStatus(4);

        r5_Interfaces_ptr_->setCatch(msg->gripper);
    }

    void R5Controller::PubState(const ros::TimerEvent&)
    {
        arx_msgs::PosCmd msg;
        // message.header.stamp = this->get_clock()->now();

        Eigen::Isometry3d transform = r5_Interfaces_ptr_->getEndPose();

        // 提取四元数和位移
        Eigen::Quaterniond quat(transform.rotation());
        Eigen::Vector3d translation = transform.translation();

        std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

        // 填充vector

        msg.x = xyzrpy[0];
        msg.y = xyzrpy[1];
        msg.z = xyzrpy[2];
        msg.roll = xyzrpy[3];
        msg.pitch = xyzrpy[4];
        msg.yaw = xyzrpy[5];
        msg.quater_x = quat.x();
        msg.quater_y = quat.y();
        msg.quater_z = quat.z();
        msg.quater_w = quat.w();

        // 发布消息
        joint_state_publisher_.publish(msg);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "r5_controller");
    ros::NodeHandle nh = ros::NodeHandle("~");
    arx::r5::R5Controller controller(nh);
    ros::spin();
    return 0;
}
