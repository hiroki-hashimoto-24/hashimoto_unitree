//unitree_go/msg/HeightMapに関する調査(visualization markerを用いて地形を表現)
//hashimoto240048@gmail.com
//2024.11.6~

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/height_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string.h>

using namespace std;
using std::placeholders::_1;

class HeightMap : public rclcpp::Node
{
public:
    HeightMap() : Node("req_sender")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 10);
        height_map_suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
            "/utlidar/height_map_array", 10, std::bind(&HeightMap::height_map_callback, this, _1));
    }

private:

    void height_map_callback(unitree_go::msg::HeightMap data)
    {
        //cout << data.data.size() << endl;
        int sec = data.stamp;
        int nanosec = data.stamp - floor(data.stamp);
        string frame_id = data.frame_id;
        int width = data.width;
        int height = data.height;
        float resolution = data.resolution;
        double origin[2] = {data.origin[0], data.origin[1]};
        //cout << "origin {" << origin[0] << ", " << origin[1] << "}" << endl;

        int wh = width*height;
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.resize(wh+1);
        //デバッグ用
        double max = -10000.0;
        double min = 10000.0;
        int count = 0;
        double ave = 0.0;

        //データの中央値を求める(地面領域を検出するために使用)
        vector<double> m_data;
        for(int j = 0; j < height; j++){
            for(int i = 0; i < width; i++){
                int jwi = j*width + i;
                //データが地上から上下2mの範囲にある場合処理を実行
                if(data.data[jwi] < 2 && data.data[jwi] > -2){
                    m_data.push_back(data.data[jwi]);
                    if(max < data.data[jwi]){
                        max = data.data[jwi];
                    }
                    if(min > data.data[jwi]){
                        min = data.data[jwi];
                    }
                    count += 1;
                }
            }
        }
        sort(m_data.begin(), m_data.end());
        double median;
        if(m_data.size()%2 == 0){
            median = (m_data[m_data.size()/2-1] + m_data[m_data.size()/2]) / 2;
        }else{
            median = m_data[m_data.size() / 2];
        }

        ave = accumulate(m_data.begin(), m_data.end(), 0.0) / m_data.size();
        //cout << "median: " << median << " ave: " << ave << endl;
        //cout << "max : " << max << " min : " << min << " count : " << count << endl;

        //箱を用いて地形を表現
        for(int j = 0; j < height; j++){
            for(int i = 0; i < width; i++){
                int jwi = j*width + i;
                //cout << setprecision(1) << data.data[j*width + i] << " ";
                marker_array.markers[jwi].header.frame_id = frame_id;
                marker_array.markers[jwi].header.stamp.sec = sec;
                marker_array.markers[jwi].header.stamp.nanosec = nanosec;
                marker_array.markers[jwi].id = jwi;
                marker_array.markers[jwi].type = visualization_msgs::msg::Marker::CUBE;
                marker_array.markers[jwi].action = visualization_msgs::msg::Marker::MODIFY;
                marker_array.markers[jwi].scale.x = resolution;
                marker_array.markers[jwi].scale.y = resolution;
                marker_array.markers[jwi].scale.z = resolution;
                marker_array.markers[jwi].pose.position.x = i*resolution + origin[0];
                marker_array.markers[jwi].pose.position.y = j*resolution + origin[1];
                double normalize = 0.0;
                double ground = 0.8;
                if(data.data[jwi] < 2 && data.data[jwi] > -2){
                    marker_array.markers[jwi].pose.position.z = data.data[jwi];
                    marker_array.markers[jwi].color.a = 1.0f;
                    normalize = (data.data[jwi] + 2) / 4;
                    //z座標が中央値と近いとき地面の色にセットする(地面を捉えた座標が中央値に存在すると仮定)
                    if(fabs(data.data[jwi]-median) < 0.05){
                        ground = 0.2;//地面の色をセット
                    }
                }else{
                    marker_array.markers[jwi].pose.position.z = 10.0;
                    marker_array.markers[jwi].color.a = 0.0f;
                }
                marker_array.markers[jwi].color.r = normalize;
                marker_array.markers[jwi].color.g = ground;
                marker_array.markers[jwi].color.b = 0.8f;
            }
        }

        //ロボットの位置を追加
        marker_array.markers[wh].header.frame_id = frame_id;
        marker_array.markers[wh].header.stamp.sec = sec;
        marker_array.markers[wh].header.stamp.nanosec = nanosec;
        marker_array.markers[wh].id = wh;
        marker_array.markers[wh].type = visualization_msgs::msg::Marker::CYLINDER;
        marker_array.markers[wh].action = visualization_msgs::msg::Marker::MODIFY;
        marker_array.markers[wh].scale.x = 0.05;
        marker_array.markers[wh].scale.y = 0.05;
        marker_array.markers[wh].scale.z = 0.1;
        marker_array.markers[wh].pose.position.x = origin[0] + width*resolution/2;
        marker_array.markers[wh].pose.position.y = origin[1] + height*resolution/2;
        marker_array.markers[wh].pose.position.z = 0.5;
        marker_array.markers[wh].color.a = 1.0f;
        marker_array.markers[wh].color.r = 1.0f;
        marker_array.markers[wh].color.g = 0.1f;
        marker_array.markers[wh].color.b = 0.1f;

        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr height_map_suber_;
};

int main(int argc, char *argv[])
{
    cout << "start height map" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<HeightMap>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}