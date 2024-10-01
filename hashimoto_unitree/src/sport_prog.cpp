//High Level制御の練習用プログラム(プログラムの置き場所の勉強) test_pkgのsport_prog2をコピーしたもの
//hashimoto240048@gmail.com
//2024.9.30~

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std;
using std::placeholders::_1;
// Create a soprt_request class for soprt commond request

class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        // the state_suber is set to subscribe "sportmodestate" topic
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&soprt_request::state_callback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&soprt_request::timer_callback, this));

        t = -1; // Runing time count
    };

private:
    void timer_callback()
    {
        t += dt;
        switch (step)
        {
        case 0:
            //1秒待つ
            wait += dt;
            if(wait>1){
                step=1;
                wait=0.0;
            }
            break;
        case 1:
            //立ち上がる
            sport_req.StandUp(req);
            break;
        case 2:
            //歩く
            walk();
            //sport_req.StandDown(req);
            req_puber->publish(req);
            break;
        case 3:
            //伏せ
            sport_req.StandDown(req);
            break;
        default:
            sport_req.StopMove(req);
            break;
        }
        //cout << "call2" << endl;
        req_puber->publish(req);
    };

    void walk(){
        // Get request messages corresponding to high-level motion commands
        float vx = 0.2;
        float vy = 0.0;
        float vyaw = 0.0;

        if(hypot(nx-px0,ny-py0) < 1.0){//歩行プログラムを開始した位置から半径1.0m移動するまで前進
            sport_req.Move(req, vx, vy, vyaw);
        }else{
            if(walk_flag==false){
                cout << "walked" << endl;
                walk_flag = true;
                wait=0.0;
            }
            wait += dt;
            sport_req.StopMove(req);
        }
        req_puber->publish(req);

        if(walk_flag == true && wait > 1.0){
            step = 3;
        }
        
        if(t>15){
            cout << "time over" << endl;
            step=3;
        }//15秒経ったら次のステップへ移行
    }

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current position of robot when t<0
        // This position is used as the initial coordinate system
        nx = data->position[0];
        ny = data->position[1];
        nz = data->position[2];
        nyaw = data->imu_state.rpy[2];

        if (t < 0)
        {
            // Get initial position
            px0 = data->position[0];
            py0 = data->position[1];
            pz0 = data->position[2];
            yaw0 = data->imu_state.rpy[2];
            //std::cout << px0 << ", " << py0 << ", " << pz0 << ", " << yaw0 << std::endl;
        }

        if(data->position[2]>0.30 && step==1){
            wait+=dt;
            if(max_height<data->position[2]) max_height = data->position[2];
            if(wait>3.0){
                step=2;
                cout << "stand up" << endl;
                cout << "max_height" << max_height << endl;
                wait = 0.0;
                sport_req.StopMove(req);
                req_puber->publish(req);
            }
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double pz0 = 0;  // initial z position
    double yaw0 = 0; // initial yaw angle

    double nx=0.0;  // now x positon
    double ny=0.0;
    double nz=0.0;
    double nyaw=0.0;

    bool stand_flag=false;

    int step=0;
    double wait=0.0;//次の動作までの待ち時間
    double max_height=0.0;

    bool walk_flag=false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    cout << "stand walk sit" << endl;
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<soprt_request>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
