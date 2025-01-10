#include "arx_r5_controller/KeyBoard.hpp"

// using namespace std::chrono_literals;

namespace arx::r5
{
    KeyBoardNode::KeyBoardNode(ros::NodeHandle nh)
    {
        // 创建发布器
        joint_cmd_publisher_ = nh.advertise<arx_r5_msg::RobotCmd>("/r5_cmd", 10);
        // // 创建订阅器

        timer_ = nh.createTimer(ros::Duration(0.01), &KeyBoardNode::Update, this);
    }

    void KeyBoardNode::Update(const ros::TimerEvent&)
    {
        message_.header.stamp = ros::Time::now();

        key_[0] = ScanKeyBoard();

        printf(":%d\r\n", key_[0]);

        /*up*/
        if (key_[0] == 65 && key_[1] == 91 && key_[2] == 27)
        {
            message_.end_pos[2] += 0.005;
            message_.mode = 4;
        }

        /*down*/
        else if (key_[0] == 66 && key_[1] == 91 && key_[2] == 27)
        {
            message_.end_pos[2] -= 0.005;
            message_.mode = 4;
        }

        /*left*/
        else if (key_[0] == 68 && key_[1] == 91 && key_[2] == 27)
        {
            message_.end_pos[1] += 0.005;
            message_.mode = 4;
        }

        /*right*/
        else if (key_[0] == 67 && key_[1] == 91 && key_[2] == 27)
        {
            message_.end_pos[1] -= 0.005;
            message_.mode = 4;
        }

        /*w*/
        else if (key_[0] == 119)
        {
            message_.end_pos[0] += 0.005;
            message_.mode = 4;
        }

        /*s*/
        else if (key_[0] == 115)
        {
            message_.end_pos[0] -= 0.005;
            message_.mode = 4;
        }

        /*a*/
        else if (key_[0] == 97)
        {
            message_.end_pos[1] += 0.005;
            message_.mode = 4;
        }

        /*d*/
        else if (key_[0] == 100)
        {
            message_.end_pos[1] -= 0.005;
            message_.mode = 4;
        }

        /*i*/
        else if (key_[0] == 105)
        {
            printf("重力补偿\n");
            message_.mode = 3;
        }

        /*n*/
        else if (key_[0] == 110)
        {
            message_.end_pos[3] -= 0.05;
            message_.mode = 4;
        }

        /*m*/
        else if (key_[0] == 109)
        {
            message_.end_pos[3] += 0.05;
            message_.mode = 4;
        }

        /*,*/
        else if (key_[0] == 44)
        {
            message_.end_pos[5] += 0.05;
            message_.mode = 4;
        }

        /*/*/
        else if (key_[0] == 47)
        {
            message_.end_pos[5] -= 0.05;
            message_.mode = 4;
        }

        /*.*/
        else if (key_[0] == 46)
        {
            message_.end_pos[4] -= 0.05;
            message_.mode = 4;
        }

        /*l*/
        else if (key_[0] == 108)
        {
            message_.end_pos[4] += 0.05;
            message_.mode = 4;
        }

        /*r*/
        else if (key_[0] == 114)
        {
            message_.end_pos[0] = 0;
            message_.end_pos[1] = 0;
            message_.end_pos[2] = 0;
            message_.end_pos[3] = 0;
            message_.end_pos[4] = 0;
            message_.end_pos[5] = 0;
            message_.mode = 1;
            message_.gripper = 0;
        }

        /*o*/
        else if (key_[0] == 111)
        {
            message_.gripper += 0.15;
        }

        /*c*/
        else if (key_[0] == 99)
        {
            message_.gripper -= 0.15;
        }

        key_[2] = key_[1];
        key_[1] = key_[0];

        // 发布消息
        ROS_INFO("Publishing RobotStatus message");
        joint_cmd_publisher_.publish(message_);
    }

    int KeyBoardNode::ScanKeyBoard()
    {
        int in;
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(STDIN_FILENO, &stored_settings); // 获得stdin 输入
        new_settings = stored_settings;            //
        new_settings.c_lflag &= (~ICANON);         //
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(STDIN_FILENO, &stored_settings); // 获得stdin 输入
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //
        in = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
        return in;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "r5_keyboard");
    ros::NodeHandle nh = ros::NodeHandle("~");
    arx::r5::KeyBoardNode controller(nh);
    ros::spin();
    return 0;
}