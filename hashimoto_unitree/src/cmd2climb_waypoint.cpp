//cmd_velをsport/requestに変更するプログラム(段差乗り越えのテスト用)
//hashimoto240048@gmail.com
//2024.11.13~

#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include "std_msgs/msg/int64.hpp"

using namespace std;
using std::placeholders::_1;

class cmd2climb : public rclcpp::Node
{
public:
    cmd2climb() : Node("req_sender")
    {
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        status_suber = this->create_subscription<std_msgs::msg::Int64>(
            "/waypoint_status", 10, std::bind(&cmd2climb::status_callback, this, _1));
    };

private:
    void status_callback(std_msgs::msg::Int64 data)
    {
        if(data.data==0){
            //上りモード
            sport_req.SwitchGait(req, 3);
        }else if(data.data==1){
            //下りモード
            sport_req.SwitchGait(req, 4);
        }else{
            //通常モード
            sport_req.SwitchGait(req, 0);
        }
        req_puber->publish(req);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr status_suber;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;
};

int main(int argc, char *argv[])
{
    cout << "start cmd2climb" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<cmd2climb>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
