//lidarの距離情報のみからしきい値によって停止命令を出す
//hashimoto240048@gmail.com
//2024.12.26~

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/int64.hpp"
#include <math.h>

using namespace std;
using std::placeholders::_1;

class LidarStop : public rclcpp::Node
{
public:
    LidarStop() : Node("lidar_stop")
    {
        setParam();
        //lidar_suber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            //"livox/lidar", 10, std::bind(&LidarStop::lidarCallback, this, _1));
        livox_suber = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "livox/lidar", 10, std::bind(&LidarStop::livoxCallback, this, _1));
        //utlidar_suber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            //"utlidar/cloud", rclcpp::QoS(10).best_effort(), std::bind(&LidarStop::utlidarCallback, this, _1));
        near_object_puber = this->create_publisher<std_msgs::msg::Int64>("/near_object", 10);
    }

private:
    void setParam(){
        this->declare_parameter<double>("dist_threshold", 0.0);
        this->declare_parameter<double>("axis_x", 0.5);
        this->declare_parameter<double>("axis_y", 0.25);
        this->declare_parameter<double>("center_x", -0.2);
        this->declare_parameter<double>("upper", 0.5);
        this->declare_parameter<double>("lower", -0.3);
        this->get_parameter<double>("dist_threshold", dist_threshold_);
        this->get_parameter<double>("axis_x", axis_x_);
        this->get_parameter<double>("axis_y", axis_y_);
        this->get_parameter<double>("center_x", center_x_);
        this->get_parameter<double>("upper", upper_);
        this->get_parameter<double>("lower", lower_);
        cout << "dist_threshold:" << dist_threshold_ << endl;
        cout << "axis_x:" << axis_x_ << endl;
        cout << "axis_y:" << axis_y_ << endl;
        cout << "center_x:" << center_x_ << endl;
        cout << "upper:" << upper_ << endl;
        cout << "lower:" << lower_ << endl;
    }

    //トピックの型によって呼び出される関数を使い分け
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg){
        std::string base_frame_id = pcl_msg->header.frame_id;
        rclcpp::Time stamp = pcl_msg->header.stamp;
        const size_t number_of_points = pcl_msg->height * pcl_msg->width;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pcl_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pcl_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pcl_msg, "z");

        double min_dist = 1.0e5;
        double min_x = 0.0;
        double min_y = 0.0;
        double min_z = 0.0;
        int count=0;
        for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z){
            double point_x = (*iter_x);
            double point_y = (*iter_y);
            double point_z = (*iter_z);

            if(!bodyPoint(point_x,point_y,point_z)){
                double dist = ellipse(point_x,point_y);
                if(ceilingPoint(point_z)!=1){
                    if(min_dist>dist){
                        min_dist = dist;
                        min_x = point_x;
                        min_y = point_y;
                        min_z = point_z;
                    }
                    if(dist < 0.0){count++;}
                }
            }
        }
        //cout << "min_dist:" << min_dist << endl;

        std_msgs::msg::Int64 data;
        data.data=0;
        if(min_dist < dist_threshold_){
            data.data=1;
            if(prev_data==0){
                cout << "min_dist x y z: " << min_dist << " " << min_x << " " << min_y << " " << min_z << endl;
            }
            prev_data=1;
        }else{
            prev_data=0;
        }
        near_object_puber->publish(data);
    }

    //トピックの型によって呼び出される関数を使い分け
    void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg){
        std::string base_frame_id = livox_msg->header.frame_id;
        rclcpp::Time stamp = livox_msg->header.stamp;
        double number_of_points = livox_msg->points.size();

        double min_dist = 1.0e5;
        double min_x = 0.0;
        double min_y = 0.0;
        double min_z = 0.0;
        int count=0;
        for (size_t i = 0; i < number_of_points; ++i){
            double point_x = livox_msg->points[i].x;
            double point_y = livox_msg->points[i].y;
            double point_z = livox_msg->points[i].z;

            if(!bodyPoint(point_x,point_y,point_z)){
                double dist = ellipse(point_x,point_y);
                if(ceilingPoint(point_z)!=1){
                    if(min_dist>dist){
                        min_dist = dist;
                        min_x = point_x;
                        min_y = point_y;
                        min_z = point_z;
                    }
                    if(dist < 0.0){count++;}
                }
            }
        }
        //cout << "count:" << count << endl;

        std_msgs::msg::Int64 data;
        data.data=0;
        if(min_dist < dist_threshold_){
        	if(count>10){
            	data.data=1;
            	if(prev_data==0){
                	cout << "min_dist x y z: " << min_dist << " " << min_x << " " << min_y << " " << min_z << endl;
            	}
            	prev_data=1;
            }
        }else{
            prev_data=0;
        }
        near_object_puber->publish(data);
    }
    
    //口元のlidarを使う(暫定ボツ)
    void utlidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg){
        std::string base_frame_id = pcl_msg->header.frame_id;
        rclcpp::Time stamp = pcl_msg->header.stamp;
        const size_t number_of_points = pcl_msg->height * pcl_msg->width;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pcl_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pcl_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pcl_msg, "z");

        double min_dist = 1.0e5;
        double min_x = 0.0;
        double min_y = 0.0;
        double min_z = 0.0;
        for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z){
            double point_x = (*iter_x);
            double point_y = (*iter_y);
            double point_z = -(*iter_z);

            if(!bodyPoint(point_x,point_y,point_z)){
                double dist = ellipse(point_x,point_y);
                if(ceilingPoint(point_z)!=1){
                    if(min_dist>dist){
                        min_dist = dist;
                        min_x = point_x;
                        min_y = point_y;
                        min_z = point_z;
                    }
                }
            }
        }

        std_msgs::msg::Int64 data;
        data.data=0;
        if(min_dist < dist_threshold_){
            data.data=1;
            if(ut_prev_data==0){
                cout << "ut min_dist x y z: " << min_dist << " " << min_x << " " << min_y << " " << min_z << endl;
            }
            ut_prev_data=1;
        }else{
            ut_prev_data=0;
        }
        near_object_puber->publish(data);
    }

    //天井を写した点(くぐり抜けができる位置の点or地面)は省く
    int ceilingPoint(double z){
        double upper = upper_;
        double lower = lower_;//地面に関するしきい値(いつか実装する)
        int ceiling = 0;
        if(z > upper || z < lower){
            ceiling = 1;
        }
        return ceiling;
    }

    //ロボット本体を写した点は削除する
    bool bodyPoint(double x, double y, double z){
        double front = 0.2;
        double back = -0.2;
        double left = 0.10;
        double right = -0.10;
        double upper = 0.2;
        double lower = -0.2;
        bool inside = false;
        if(back < x && x < front){
            if(right < y && y < left){
                if(lower < z && z < upper){
                    inside = true;
                }
            }
        }
        return inside;
    }

    //当たり判定(楕円)
    double ellipse(double x, double y){
        double axis_x = axis_x_;
        double axis_y = axis_y_;
        double center_x = center_x_;
        double d = pow((x-center_x)/axis_x, 2) + pow(y/axis_y, 2) - 1;
        return d;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_suber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr utlidar_suber;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_suber;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr near_object_puber;

    double dist_threshold_;
    double axis_x_;
    double axis_y_;
    double center_x_;
    double upper_;
    double lower_;

    int prev_data=0;
    int ut_prev_data=0;
};

int main(int argc, char *argv[])
{
    cout << "start lidar_stop" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<LidarStop>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
