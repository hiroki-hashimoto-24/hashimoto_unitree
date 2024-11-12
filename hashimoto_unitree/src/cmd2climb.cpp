//cmd_velをsport/requestに変更するプログラム
//hashimoto240048@gmail.com
//2024.11.4~

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using std::placeholders::_1;

class cmd2climb : public rclcpp::Node
{
public:
    cmd2climb() : Node("req_sender")
    {
        // the state_suber is set to subscribe "cmd_vel" topic
        cmd_suber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&cmd2climb::cmd_callback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&cmd2climb::timer_callback, this));
    };

private:

    void cmd_callback(geometry_msgs::msg::Twist data)
    {
        float vx = data.linear.x;
        float vy = data.linear.y;
        float vyaw = data.angular.z;

        if(fabs(vx) < MIN_SPEED && fabs(vy) < MIN_SPEED && fabs(vyaw) < MIN_SPEED){
            sport_req.StopMove(req);
            if(debug_flag == 0){
                cout << "stop move : under min spead" << endl;
                debug_flag = 1;
            }
        }else if(fabs(vx) > MAX_SPEED || fabs(vy) > MAX_SPEED || fabs(vyaw) > MAX_SPEED){
            sport_req.StopMove(req);
            if(debug_flag == 0){
                cout << "stop move : over max spead" << endl;
                debug_flag = 1;
            }
        }else{
            sport_req.Move(req, vx, vy, vyaw);
            sport_req.SwitchGait(req, 3);
            //cout << req.header.identity.api_id << endl;
            if(debug_flag == 1){
                debug_flag = 0;
            }
        }
        req_puber->publish(req);

        timer_flag = 1;
    }

    void timer_callback()
    {
        if(timer_flag == 0){
            t += dt;
            if(t > 1.0){
                if(timer_debug_flag == 0){
                    sport_req.StopMove(req);
                    cout << "cmd_vel not received for 1.0 seconds" << endl;
                    timer_debug_flag = 1;
                }
            }
        }else{
            t = 0.0;
            timer_debug_flag = 0;
        }
        timer_flag = 0;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    int debug_flag = 0;//デバッグメッセージのフラグ
    const float MAX_SPEED = 1.0;//ロボットの速度制限
    const float MIN_SPEED = 0.01;//ロボットの停止モーションしきい値

    double t = 0.0;
    double dt = 0.1; //control time step
    int timer_flag = 0;//最後の速度命令呼び出しから一定時間たったら停止するためのフラグ
    int timer_debug_flag = 0;//デバッグメッセージのフラグ
};

int main(int argc, char *argv[])
{
    cout << "start cmd2climb" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<cmd2climb>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
