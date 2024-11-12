//cmd_velをsport/requestに変更するプログラム(obstacle_detectorによる安全停止つき)
//hashimoto240048@gmail.com
//2024.11.8~

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
        // the near_object_suber is set to subcribe "near_object" topic
        near_object_suber = this->create_subscription<std_msgs::msg::Int64>(
            "/near_object", 10, std::bind(&cmd2sport::object_callback, this, _1));
        // the req_puber is set to publish "/api/sport/request" topic
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    };

private:

    void cmd_callback(geometry_msgs::msg::Twist data)
    {
        float vx = data.linear.x;
        float vy = data.linear.y;
        float vyaw = data.angular.z;

        if(stop_flag == 1){
            sport_req.StopMove(req);
            if(debug_flag != 1){
                cout << "stop move : near object" << endl;
                debug_flag = 1;
            }
        }else if(fabs(vx) < MIN_SPEED && fabs(vy) < MIN_SPEED && fabs(vyaw) < MIN_SPEED){
            sport_req.StopMove(req);
            if(debug_flag != 1){
                cout << "stop move : under min spead" << endl;
                debug_flag = 1;
            }
        }else if(fabs(vx) > MAX_SPEED || fabs(vy) > MAX_SPEED || fabs(vyaw) > MAX_SPEED){
            sport_req.StopMove(req);
            if(debug_flag != 1){
                cout << "stop move : over max spead" << endl;
                debug_flag = 1;
            }
        }else{
            sport_req.Move(req, vx, vy, vyaw);
            if(debug_flag != 0){
                cout << "move start" << endl;
                debug_flag = 0;
            }
        }
        
        req_puber->publish(req);
    }

    void object_callback(std_msgs::msg::Int64 data){
        if(int(data.data) == 1){
            stop_flag = 1;
        }else{
            stop_flag = 0;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_suber;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr near_object_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    int debug_flag = 2;
    const float MAX_SPEED = 1.0;
    const float MIN_SPEED = 0.01;

    int stop_flag = 0;
};

int main(int argc, char *argv[])
{
    cout << "start cmd2sport" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<cmd2sport>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
