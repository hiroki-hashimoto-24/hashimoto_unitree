//cmd_velをsport/requestに変更するプログラム｀
//hashimoto240048@gmail.com
//2024.10.8~

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using std::placeholders::_1;

class cmd2sport : public rclcpp::Node
{
public:
    cmd2sport() : Node("req_sender")
    {
        // the state_suber is set to subscribe "cmd_vel" topic
        cmd_suber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&cmd2sport::cmd_callback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    };

private:

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr data)
    {
        float vx = data->linear.x;
        float vy = data->linear.y;
        float vyaw = data->angular.z;

        if(fabs(vx) < MIN_SPEED && fabs(vy) < MIN_SPEED && fabs(vyaw) < MIN_SPEED){
            sport_req.StopMove(req);
            if(debag_flag == 0){
                cout << "stop move : under min spead" << endl;
                debag_flag = 1;
            }
        }else if(fabs(vx) > MAX_SPEED || fabs(vy) > MAX_SPEED || fabs(vyaw) > MAX_SPEED){
            sport_req.StopMove(req);
            if(debag_flag == 0){
                cout << "stop move : over max spead" << endl;
                debag_flag = 1;
            }
        }else{
            sport_req.Move(req, vx, vy, vyaw);
            if(debag_flag == 1){
                debag_flag = 0;
            }
        }
        
        req_puber->publish(req);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    int debag_flag = 0;
    const float MAX_SPEED = 1.0;
    const float MIN_SPEED = 0.01;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    cout << "start" << endl;

    rclcpp::spin(std::make_shared<cmd2sport>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
