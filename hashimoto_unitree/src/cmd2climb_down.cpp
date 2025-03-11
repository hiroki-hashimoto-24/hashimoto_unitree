//cmd_velをsport/requestに変更するプログラム(上り下りの命令の変更があった場合1秒止まる)
//hashimoto240048@gmail.com
//2024.12.06~

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std;
using std::placeholders::_1;

class cmd2sport : public rclcpp::Node
{
public:
    cmd2sport() : Node("req_sender")
    {
        // the cmd_suber is set to subscribe "cmd_vel" topic
        cmd_suber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&cmd2sport::cmd_callback, this, _1));
        // the req_puber is set to publish "/api/sport/request" topic
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&cmd2sport::timerCallback, this));
        status_suber = this->create_subscription<std_msgs::msg::Int64>(
            "/climb_status", 10, std::bind(&cmd2sport::status_callback, this, _1));
    };

private:

    void cmd_callback(geometry_msgs::msg::Twist::SharedPtr data)
    {
        float vx = data->linear.x;
        float vy = data->linear.y;
        float vyaw = data->angular.z;
        
        if(timer_flag==0){
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
                if(debug_flag == 1){
                    debug_flag = 0;
                }
            }
            
            req_puber->publish(req);
        }
    }

    void status_callback(std_msgs::msg::Int64::SharedPtr data)
    {
        if(prev_climb!=data->data){
            timer_flag = 1;
        }
        prev_climb=data->data;
        if(timer_flag == 1){
            sport_req.StopMove(req);
            req_puber->publish(req);

            if(data->data==1){
                //通常モード
                sport_req.SwitchGait(req, 1);
            }else if(data->data==3){
                //上りモード
                sport_req.SwitchGait(req, 3);
            }else if(data->data==4){
                //下りモード
                sport_req.SwitchGait(req, 4);
            }else{
                //例外処理
                //通常モード(暫定)
                sport_req.SwitchGait(req, 1);
                //sport_req.StopMove(req);
            }
            req_puber->publish(req);
        }
    }

    //待機ステップで経過時間をカウントするためのコールバック関数
    void timerCallback()
    {
        if(timer_flag == 1){
            t += dt;
            if(t > 1.0){
                timer_flag = 0;
                cout << "departure" << endl;
            }
            if(timer_debug_flag == 0){
                cout << "park" << endl;
                timer_debug_flag = 1;
            }
        }else{
            t = 0.0;
            timer_flag = 0;
            timer_debug_flag = 0;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr status_suber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    int debug_flag = 0;
    const float MAX_SPEED = 1.0;
    const float MIN_SPEED = 0.01;

    double t = 0.0;
    double dt = 0.1; //control time step
    int timer_flag = 0;//最後の速度命令呼び出しから一定時間たったら停止するためのフラグ
    int timer_debug_flag = 0;//デバッグメッセージのフラグ

    int prev_climb = 0;//直前の走行モード
};

int main(int argc, char *argv[])
{
    cout << "start cmd2sport" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<cmd2sport>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
