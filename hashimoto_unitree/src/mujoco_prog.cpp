// これはUnitreeの練習用のプログラム おすわりする
// 240048 hashimoto
// 2024.7.17~
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "motor_crc.h"

#include "unitree_go/msg/sport_mode_state.hpp"
#include "common/ros2_sport_client.h"

#include <cmath>

using std::placeholders::_1;
using namespace std;

// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node
{
public:
    low_level_cmd_sender() : Node("low_level_cmd_sender")
    {
        // the cmd_puber is set to subscribe "/lowcmd" topic
        cmd_puber = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&low_level_cmd_sender::state_callback, this, _1));

        // The timer is set to 200Hz, and bind to low_level_cmd_sender::timer_callback function
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&low_level_cmd_sender::timer_callback, this));

        // Initialize lowcmd
        init_cmd();

        //eular 2 radian
        for(int i=0; i<12; i++){
            stand_up_joint_pos[i] = euler_stand_up_joint_pos[i]*M_PI/180;
            stand_down_joint_pos[i] = euler_stand_down_joint_pos[i]*M_PI/180;
            osuwari_joint_pos[i] = euler_osuwari_joint_pos[i]*M_PI/180;
            ote_joint_pos[i] = euler_ote_joint_pos[i]*M_PI/180;
        }
    }

private:
    void timer_callback()
    {
        runing_time += dt;
        switch (motion)
        {
            //立ち上がり
            case 0:
                phase = tanh((runing_time - 3.0*motion) / 1.2);
                for (int i = 0; i < 12; i++)
                {
                    low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                    low_cmd.motor_cmd[i].kd = 3.5;
                    low_cmd.motor_cmd[i].tau = 0;
                }
                //3秒立ったら次のモーションへ以降(割り算のあまりを使用するためキャストしている)
                if(static_cast<int>(static_cast<float>(runing_time*1000))%3000 == 0){
                    motion=1;
                    cout << "motion: " << motion << endl;
                }
                break;
            //しゃがみ込み
            case 1:
                phase = tanh((runing_time - 3.0*motion) / 1.2);
                for (int i = 0; i < 12; i++)
                {
                    low_cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                    low_cmd.motor_cmd[i].kd = 3.5;
                    low_cmd.motor_cmd[i].tau = 0;
                }
                if(static_cast<int>(static_cast<float>(runing_time*1000))%3000 == 0){
                    motion=2;
                    cout << "motion: " << motion << endl;
                }
                break;
            //立ち上がり
            case 2:
                phase = tanh((runing_time - 3.0*motion) / 1.2);
                for (int i = 0; i < 12; i++)
                {
                    low_cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                    low_cmd.motor_cmd[i].kd = 3.5;
                    low_cmd.motor_cmd[i].tau = 0;
                }
                if(static_cast<int>(static_cast<float>(runing_time*1000))%3000 == 0){
                    motion=3;
                    cout << "motion: " << motion << endl;
                }
                break;
            //おすわり
            case 3:
                phase = tanh((runing_time - 3.0*motion) / 1.2);
                for (int i = 0; i < 12; i++)
                {
                    low_cmd.motor_cmd[i].q = phase * osuwari_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                    low_cmd.motor_cmd[i].kd = 3.5;
                    low_cmd.motor_cmd[i].tau = 0;
                }
                if(static_cast<int>(static_cast<float>(runing_time*1000))%3000 == 0){
                    motion=4;
                    cout << "motion: " << motion << endl;
                }
                break;
            //お手
            case 4:
                phase = tanh((runing_time - 3.0*motion) / 1.2);
                for (int i = 0; i < 12; i++)
                {
                    low_cmd.motor_cmd[i].q = phase * ote_joint_pos[i] + (1 - phase) * osuwari_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                    low_cmd.motor_cmd[i].kd = 3.5;
                    low_cmd.motor_cmd[i].tau = 0;
                }
                if(static_cast<int>(static_cast<float>(runing_time*1000))%3000 == 0){
                    motion=5;
                    cout << "motion: " << motion << endl;
                }
                break;
            //おすわり
            case 5:
                phase = tanh((runing_time - 3.0*motion) / 1.2);
                for (int i = 0; i < 12; i++)
                {
                    low_cmd.motor_cmd[i].q = phase * osuwari_joint_pos[i] + (1 - phase) * ote_joint_pos[i];
                    low_cmd.motor_cmd[i].dq = 0;
                    low_cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0;
                    low_cmd.motor_cmd[i].kd = 3.5;
                    low_cmd.motor_cmd[i].tau = 0;
                }
                if(static_cast<int>(static_cast<float>(runing_time*1000))%3000 == 0){
                    motion=6;
                    cout << "motion: " << motion << endl;
                }
                break;
            default:
                break;
        }
        get_crc(low_cmd);            // Check motor cmd crc
        cmd_puber->publish(low_cmd); // Publish lowcmd message
    }

    void init_cmd()
    {

        for (int i = 0; i < 20; i++)
        {
            low_cmd.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
            low_cmd.motor_cmd[i].q = PosStopF;
            low_cmd.motor_cmd[i].kp = 0;
            low_cmd.motor_cmd[i].dq = VelStopF;
            low_cmd.motor_cmd[i].kd = 0;
            low_cmd.motor_cmd[i].tau = 0;
        }
    }

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current position of robot when t<0
        // This position is used as the initial coordinate system

        if (runing_time > 0)
        {
            //cout << "pz:" << data->position[2] << endl;
            if(max_z < data->position[2]){
                max_z = data->position[2];
            }

            if (data->position[2] > 0.35 && stand_flag == false){
                cout << "stand up" << endl;
                stand_flag = true;

                cout << "stand pos: " << endl;
                for(int i = 0; i<12; i++){
                    cout << stand_up_joint_pos[i]*180/M_PI << ", ";
                    if (i==5){
                        cout << endl;
                    }
                }
                cout << endl;

                cout << "stit pos: " << endl;
                for(int i = 0; i<12; i++){
                    cout << stand_down_joint_pos[i]*180/M_PI << ", ";
                    if (i==5){
                        cout << endl;
                    }
                }
                cout << endl;

                cout << "osuwari pos: " << endl;
                for(int i = 0; i<12; i++){
                    cout << osuwari_joint_pos[i]*180/M_PI << ", ";
                    if (i==5){
                        cout << endl;
                    }
                }
                cout << endl;
            }

            if (motion == 6 && stand_flag == true){
                stand_flag = false;
                cout << "max z:" << max_z << endl;
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;                             // ROS2 timer
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_puber; // ROS2 Publisher

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber; // ROS2 Subscriber

    unitree_go::msg::LowCmd low_cmd;

    double stand_up_joint_pos[12];
    double stand_down_joint_pos[12];
    double osuwari_joint_pos[12];
    double ote_joint_pos[12];

    double euler_stand_up_joint_pos[12] = {0.327656, 34.8824, -69.7651, -0.327656, 34.8824, -69.7651,
                                           0.327656, 34.8824, -69.7651, -0.327656, 34.8824, -69.7651};
    double euler_stand_down_joint_pos[12] = {2.7127, 70.008, -140.017, -2.7127, 70.008, -140.017,
                                             2.7127, 70.008, -140.017, -2.7127, 70.008, -140.017};
    double euler_osuwari_joint_pos[12] = {0.327656, 34.8824, -69.7651, -0.327656, 34.8824, -69.7651,
                                          2.7127, 70.008, -140.017, -2.7127, 70.008, -140.017};
    double euler_ote_joint_pos[12] = {0.327656, -14.8824, -69.7651, -20.327656, 34.8824, -69.7651,
                                      2.7127, 70.008, -140.017, -2.7127, 70.008, -140.017};

    //double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    //                                 0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    //double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
    //                                   0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    
    //stand pos: 
    //0.327656, 34.8824, -69.7651, -0.327656, 34.8824, -69.7651, 
    //0.327656, 34.8824, -69.7651, -0.327656, 34.8824, -69.7651, 
    //stit pos: 
    //2.7127, 70.008, -140.017, -2.7127, 70.008, -140.017,
    //2.7127, 70.008, -140.017, -2.7127, 70.008, -140.017,

    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    double max_z=0.0;

    bool stand_flag = false;
    int motion=0;
};

int main(int argc, char **argv)
{   
    std::cout << "Press enter to start";
    std::cin.get();
    
    rclcpp::init(argc, argv);                             // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals
    auto node = std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make share with low_level_cmd_sender class
    rclcpp::spin(node);                                   // Run ROS2 node
    rclcpp::shutdown();                                   // Exit
    return 0;
}