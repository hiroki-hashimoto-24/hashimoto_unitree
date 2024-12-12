//段差乗り越えに関するプログラム(判定を横に広くして片足だけが乗り上げるパターンにも対応する)
//hashimoto240048@gmail.com
//2024.12.09~

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/height_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "unitree_go/msg/sport_mode_state.hpp"
#include "std_msgs/msg/int64.hpp"
#include <map>

using namespace std;
using std::placeholders::_1;
using namespace cv;

class HeightMap : public rclcpp::Node
{
public:
    HeightMap() : Node("req_sender")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 4);
        height_map_suber_ = this->create_subscription<unitree_go::msg::HeightMap>(
            "/utlidar/height_map_array", 4, std::bind(&HeightMap::height_map_callback, this, _1));
        state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&HeightMap::stateCallback, this, std::placeholders::_1));
        status_pub_ = this->create_publisher<std_msgs::msg::Int64>("climb_status", 10);
    }

private:

    void height_map_callback(unitree_go::msg::HeightMap data)
    {
        //cout << data.data.size() << endl;
        int sec = data.stamp;
        int nanosec = data.stamp - floor(data.stamp);
        string frame_id = data.frame_id;
        int height = data.height;
        int width = data.width;
        float resolution = data.resolution;
        double origin[2] = {data.origin[0], data.origin[1]};
        //cout << "origin {" << origin[0] << ", " << origin[1] << "}" << endl;

        if(debug_flag == 0){
            cout << "h w r : " << height << " " << width << " " << resolution << endl;
            debug_flag = 1;
        }

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

        //カーネル密度推定による最瀕値の予測。ビン幅が少しづつ重なった状態での最瀕値の推定ができると思って使っている
        //何ｍおきに推定するか
        double scale = 0.01;
        //バンド幅
        double bandwidth = 0.05;
        //上下何メートル以内に地面があると予想するか
        double range = 1.0;
        int bin = range*2/scale;
        //cout << bin << endl;
        vector<double> points(bin);

        //-0.5m~0.5m(range)の区間を0.01m置きに推定
        for(int i=0;i<bin;i++){
            points[i] = i*scale - range;
        }

        vector<double> densities = kernel_density_estimation(m_data, points, bandwidth);

        double densities_max=0.0;
        int index=0;
        for(int i=0;i<bin;i++){
            if(densities_max < densities[i]){
                densities_max = densities[i];
                index = i;
            }
        }

        double kde = points[index];

        sort(m_data.begin(), m_data.end());
        double median;
        if(m_data.size()%2 == 0){
            median = (m_data[m_data.size()/2-1] + m_data[m_data.size()/2]) / 2;
        }else{
            median = m_data[m_data.size() / 2];
        }
        ave = accumulate(m_data.begin(), m_data.end(), 0.0) / m_data.size();

        //cout << "max : " << max << " min : " << min << " count : " << count << endl;
        //cout << "kde: " << kde << " median: " << median << " ave: " <<  ave << endl;
        //ここまで

        //データを画像にしてみる
        Mat img(Size(height, width), CV_8UC1);
        double height_range = 0.5;
        for(int j = 0; j < height; j++){
            for(int i = 0; i < width; i++){
                int jwi = j*width + i;
                if(data.data[jwi] < height_range && data.data[jwi] > -height_range){
                    img.at<unsigned char>(i,j) = int((data.data[jwi]+height_range)/(2*height_range) * 255);
                }else{
                    img.at<unsigned char>(i,j) = 255;
                }
            }
        }

        //中央値フィルタ
        Mat med;
        medianBlur(img, med, 3);

        //デバッグ用
        //int mid_col = int((kde+height_range)/(2*height_range) * 255);
        //cout << "mid_col:" << mid_col << endl;

        //輪郭を重ねる土台
        Mat result_thr = Mat::zeros(img.size(), CV_8UC3);
        for(int j = 0; j < height; j++){
            for(int i = 0; i < width; i++){
                int gray = med.at<unsigned char>(i,j);
                result_thr.at<Vec3b>(i, j) = Vec3b(gray, gray, gray);
            }
        }

        //vt = 0.3mで計算->5ピクセル先
        //輝度差1あたり0.004mくらい
        double vt = 0.6;
        //double theta = 0.0;
        Point p_sh(width/2-1 + int(-0.3*cos(theta)/0.06), height/2-1 + int(-0.3*sin(theta)/0.06));
        Point p_dh(width/2-1 + int(vt*cos(theta)/0.06), height/2-1 + int(vt*sin(theta)/0.06));
        double sh = med.at<unsigned char>(p_sh.x, p_sh.y);//自分の高さ self height
        double dh = med.at<unsigned char>(p_dh.x, p_dh.y);//行き先の高さ destination height
        //cout << "p_sh" << p_sh << ":" << sh << " p_dh" << p_dh << ":" << dh << endl;
        double oh = 10;//同じ高さかどうかのしきい値(0.04m) onazi height
        double ah = 40;//安全に登ることができる限界(0.16m) anzen height

        //前足後ろ足の概念を導入
        Point p1(width/2-1 + int(-0.3*cos(theta)/0.06), height/2-1 + int(-0.3*sin(theta)/0.06));
        Point p2(width/2-1, height/2-1);
        Point p3(width/2-1 + int(0.3*cos(theta)/0.06), height/2-1 + int(0.3*sin(theta)/0.06));
        Point p4(width/2-1 + int(0.6*cos(theta)/0.06), height/2-1 + int(0.6*sin(theta)/0.06));
        double h1 = med.at<unsigned char>(p1.x, p1.y);
        double h2 = med.at<unsigned char>(p2.x, p2.y);
        double h3 = med.at<unsigned char>(p3.x, p3.y);
        double h4 = med.at<unsigned char>(p4.x, p4.y);

        //cout << "h1 h2 h3 h4: " << h1 << " " << h2 << " " << h3 << " " << h4 << endl;

        //左右足の概念を導入
        double leg_width = 0.15;
        Point lp1(width/2-1 + int(-0.3*cos(theta)/0.06) + int(leg_width*sin(theta)/0.06), height/2-1 + int(-0.3*sin(theta)/0.06) + int(leg_width*cos(theta)/0.06));
        Point lp2(width/2-1 + int(leg_width*sin(theta)/0.06), height/2-1 + int(leg_width*cos(theta)/0.06));
        Point lp3(width/2-1 + int(0.3*cos(theta)/0.06)+ int(leg_width*sin(theta)/0.06), height/2-1 + int(0.3*sin(theta)/0.06) + int(leg_width*cos(theta)/0.06));
        Point lp4(width/2-1 + int(0.6*cos(theta)/0.06) + int(leg_width*sin(theta)/0.06), height/2-1 + int(0.6*sin(theta)/0.06) + int(leg_width*cos(theta)/0.06));
        double lh1 = med.at<unsigned char>(lp1.x, lp1.y);
        double lh2 = med.at<unsigned char>(lp2.x, lp2.y);
        double lh3 = med.at<unsigned char>(lp3.x, lp3.y);
        double lh4 = med.at<unsigned char>(lp4.x, lp4.y);
        Point rp1(width/2-1 + int(-0.3*cos(theta)/0.06) + int(-leg_width*sin(theta)/0.06), height/2-1 + int(-0.3*sin(theta)/0.06) + int(-leg_width*cos(theta)/0.06));
        Point rp2(width/2-1 + int(-leg_width*sin(theta)/0.06), height/2-1 + int(-leg_width*cos(theta)/0.06));
        Point rp3(width/2-1 + int(0.3*cos(theta)/0.06)+ int(-leg_width*sin(theta)/0.06), height/2-1 + int(0.3*sin(theta)/0.06) + int(-leg_width*cos(theta)/0.06));
        Point rp4(width/2-1 + int(0.6*cos(theta)/0.06) + int(-leg_width*sin(theta)/0.06), height/2-1 + int(0.6*sin(theta)/0.06) + int(-leg_width*cos(theta)/0.06));
        double rh1 = med.at<unsigned char>(rp1.x, rp1.y);
        double rh2 = med.at<unsigned char>(rp2.x, rp2.y);
        double rh3 = med.at<unsigned char>(rp3.x, rp3.y);
        double rh4 = med.at<unsigned char>(rp4.x, rp4.y);
        
        //モード切替の塗り分け
        int color_b=0,color_g=0,color_r=0;

        //条件分岐
        // if(fabs(h1-h3) <= oh && fabs(h2-h4) <= oh){
        //     cout << "0 FM" << endl;//平地モード
        //     color_r=255;
        //     status.data = 0;
        // }else if(((oh < fabs(h1-h3) && fabs(h1-h3) < ah && h1-h3<0)) || ((oh < fabs(h2-h4) && fabs(h2-h4) < ah && h2-h4<0))){
        //     cout << "3 CM" << endl;//上りモード
        //     color_g=255;
        //     status.data = 3;
        // }else if(((oh < fabs(h1-h3) && fabs(h1-h3) < ah && h1-h3>0)) || ((oh < fabs(h2-h4) && fabs(h2-h4) < ah && h2-h4>0))){
        //     cout << "4 DM" << endl;//下りモード
        //     color_b=255;
        //     status.data = 4;
        // }else{
        //     cout << "10 SM" << endl;//例外処理
        //     color_r=255;
        //     color_g=255;
        //     status.data = 10;
        // }

        //条件分岐(左右足の概念を追加)
        bool cond[3][3];//conditional:各条件を変数に格納してif文を整理
        cond[0][0] = (fabs(h1-h3) <= oh && fabs(h2-h4) <= oh);//前後の足が共に平地
        cond[0][1] = (fabs(lh1-lh3) <= oh && fabs(lh2-lh4) <= oh);
        cond[0][2] = (fabs(rh1-rh3) <= oh && fabs(rh2-rh4) <= oh);
        cond[1][0] = (((oh < fabs(h1-h3) && fabs(h1-h3) < ah && h1-h3<0)) || ((oh < fabs(h2-h4) && fabs(h2-h4) < ah && h2-h4<0)));//前後どちらかの足が上り区間
        cond[1][1] = (((oh < fabs(lh1-lh3) && fabs(lh1-lh3) < ah && lh1-lh3<0)) || ((oh < fabs(lh2-lh4) && fabs(lh2-lh4) < ah && lh2-lh4<0)));
        cond[1][2] = (((oh < fabs(rh1-rh3) && fabs(rh1-rh3) < ah && rh1-rh3<0)) || ((oh < fabs(rh2-rh4) && fabs(rh2-rh4) < ah && rh2-rh4<0)));
        cond[2][0] = ((oh < fabs(h1-h3) && fabs(h1-h3) < ah && h1-h3>0)) || ((oh < fabs(h2-h4) && fabs(h2-h4) < ah && h2-h4>0));//前後どちらかの足が下り区間
        cond[2][1] = ((oh < fabs(lh1-lh3) && fabs(lh1-lh3) < ah && lh1-lh3>0)) || ((oh < fabs(lh2-lh4) && fabs(lh2-lh4) < ah && lh2-lh4>0));
        cond[2][2] = ((oh < fabs(rh1-rh3) && fabs(rh1-rh3) < ah && rh1-rh3>0)) || ((oh < fabs(rh2-rh4) && fabs(rh2-rh4) < ah && rh2-rh4>0));

        if(cond[0][0] && cond[0][1] && cond[0][2]){
            //cout << "0 FM" << endl;//平地モード
            color_r=255;
            status.data = 0;
        }else if(cond[1][0] || cond[1][1] || cond[1][2]){
            //cout << "3 CM" << endl;//上りモード
            color_g=255;
            status.data = 3;
        }else if(cond[2][0] || cond[2][1] || cond[2][2]){
            //cout << "4 DM" << endl;//下りモード
            color_b=255;
            status.data = 4;
        }else{
            //cout << "10 SM" << endl;//例外処理
            color_r=255;
            color_g=255;
            status.data = 10;
        }

        //デバッグメッセージ
        if(detailed_log){
            cout << "p_sh" << p_sh << ":" << sh << " p_dh" << p_dh << ":" << dh << " ";
            map<int, string> mp{
                {0,"FM"},
                {3,"CM"},
                {4,"DM"},
                {10,"SM"}
            };
            cout << status.data << ":" << mp[status.data] << endl;
        }

        //白黒反転(視認性のために)
        Mat bn;
        bitwise_not(result_thr,bn);

        //代表座標を塗り分け
        result_thr.at<Vec3b>(width/2-1, height/2-1) = Vec3b(color_b, color_g, color_r);
        bn.at<Vec3b>(width/2-1, height/2-1) = Vec3b(color_b, color_g, color_r);

        //画像出力
        imgOutput(img, "img");
        imgOutput(med, "median");
        imgOutput(result_thr, "result_thr");
        imgOutput(bn, "bitwise_not");

	    status_pub_->publish(status);
    }

    void stateCallback(const unitree_go::msg::SportModeState::SharedPtr data) {
        theta = data->imu_state.rpy[2];
        //cout << "theta:" << theta << endl;
    }

    //ガウシアンカーネル
    double gaussian_kernel(double x){
        return (1.0/sqrt(2 * M_PI)) * exp(-(x*x)/2);
    }

    //カーネル密度推定
    vector<double> kernel_density_estimation(vector<double>& data, vector<double> points, double bandwidth){
        vector<double> densities(points.size(), 0.0);
        double sum_densities = 0.0;
        for(int i=0; i<points.size(); i++){
            double sum = 0.0;
            for(int j=0; j<data.size(); j++){
                sum += gaussian_kernel((points[i] - data[j])/ bandwidth);
            }
            densities[i] = sum / (data.size() * bandwidth);
            //cout << points[i] << " : " << densities[i] << endl;
            sum_densities += densities[i];
        }
        //cout << "sum densities : " << sum_densities << endl;
        return densities;
    }

    //画像表示
    void imgOutput(const Mat src, string image_name){
        double enl = 4.0;//enlarge
        Mat enl_img;
        rotate(src, enl_img, ROTATE_90_COUNTERCLOCKWISE);
        resize(enl_img, enl_img, Size(), enl, enl);
        imshow(image_name, enl_img);
        cv::waitKey(1);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr height_map_suber_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr status_pub_;

    std_msgs::msg::Int64 status;//走行ステータスの型の宣言

    int preb_size = 0;
    int debug_flag = 0;
    int approach_flag = 0;
    double theta = 0.0;
    bool detailed_log=true;
};

int main(int argc, char *argv[])
{
    cout << "start height map!" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<HeightMap>()); //Run ROS2 node
    rclcpp::shutdown();
    return 0;
}