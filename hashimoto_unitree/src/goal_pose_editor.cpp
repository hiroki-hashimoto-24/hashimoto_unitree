//geometry_msgs/msg/PoseStamped型のトピックを受け取ってファイルに書き出す
//launchで起動するとstd::cinがうまく動かない。原因調査中
//hashimoto240048@gmail.com
//2024.10.25~

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <iomanip>
#include <iostream>
#include <fstream>

using namespace std;
using std::placeholders::_1;

bool g_key_flag = false;

typedef struct{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
}Pose;

std::string replaceOtherStr(std::string &replacedStr, std::string from, std::string to) {
    const unsigned int pos = replacedStr.find(from);
    const int len = from.length();

    if (pos == std::string::npos || from.empty()) {
        return replacedStr;
    }

    return replacedStr.replace(pos, len, to);
}

class GoalPose : public rclcpp::Node
{
public:
    GoalPose() : Node("req_sender")
    {
        setParam();
        goal_pose_suber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&GoalPose::goal_pose_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&GoalPose::timer_callback, this));
    }

private:
    void setParam(){
        string file_path;
        string file_name;
        string default_file_path = ament_index_cpp::get_package_prefix("hashimoto_unitree");
        default_file_path = replaceOtherStr(default_file_path,"install","src") + "/nav2_waypoint/";
        this->declare_parameter<string>("file_path", default_file_path);
        this->get_parameter<string>("file_path", file_path);
        this->declare_parameter<string>("file_name", "sample.csv");
        this->get_parameter<string>("file_name", file_name);
        path_file_name_ = file_path + file_name;
        cout << "path file name : " << path_file_name_ << endl;
    }

    void goal_pose_callback(geometry_msgs::msg::PoseStamped data)
    {
        Pose pose;
        pose.x =  data.pose.position.x;
        pose.y =  data.pose.position.y;
        pose.z =  data.pose.position.z;
        pose.qx =  data.pose.orientation.x;
        pose.qy =  data.pose.orientation.y;
        pose.qz =  data.pose.orientation.z;
        pose.qw =  data.pose.orientation.w;

        cout << fixed;
        cout << "x y z qx qy qz qw : " << setprecision(3) << pose.x << " " << pose.y << " " << pose.z << " " << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw << endl;

        v_pose_.push_back(pose);
    }

    void timer_callback(){
        if(g_key_flag){
            //output massage
            for(int i = 0; i < v_pose_.size(); i++){
                cout << fixed << setprecision(3) << v_pose_[i].x << "," << v_pose_[i].y << "," << v_pose_[i].z << "," << v_pose_[i].qx << "," << v_pose_[i].qy << "," << v_pose_[i].qz << "," << v_pose_[i].qw << endl;
            }

            ofstream fs(path_file_name_);
            if(!fs){
                cout << "cannot open the file" << endl;
            }else{
                fs << "x,y,z,qx,qy,qz,qw" << endl;
                for(int i = 0; i < v_pose_.size(); i++){
                    fs << fixed << setprecision(3) << v_pose_[i].x << "," << v_pose_[i].y << "," << v_pose_[i].z << "," << v_pose_[i].qx << "," << v_pose_[i].qy << "," << v_pose_[i].qz << "," << v_pose_[i].qw << endl;
                }
                fs.close();
                cout << "edit" << endl;
            }
            g_key_flag =  false;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_suber;
    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer

    string path_file_name_;
    vector<Pose> v_pose_;

    double dt = 0.1; //control time step
};

void keyInput(){
    if(!g_key_flag){
        string key;
        cin >> key;
        if(key == "e"){
            //cout << "input e" << endl;
            g_key_flag = true;
        }
    }
}

int main(int argc, char *argv[])
{
    cout << "start goal_pose_editor" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp

    cout << "input key e -> edit file" << endl;
    thread inputThread(keyInput);
    while (inputThread.joinable()) {
        rclcpp::spin(std::make_shared<GoalPose>()); //Run ROS2 node
        this_thread::sleep_for(chrono::seconds(1));
        cout << "waiting for key-in..." << endl;
    }

    rclcpp::shutdown();
    return 0;
}
