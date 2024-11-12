//obstacle_detectorを用いた障害物の衝突判定プログラム
//hashimoto240048@gmail.com
//2024.11.7~

#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "obstacle_detector/msg/obstacles.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std;
using std::placeholders::_1;

#define FRONT_LINE -0.3//レーザー位置より後ろを見ないようにする(0.1のとき前方0.1m以上前に存在する物体のみを認識する)
#define IGNORE_WALL 0.2//一定値よりも小さい壁は無視する
//#define IGNORE_CIRCLE 0.2//一定値よりも小さい円オブジェクトは無視する
#define IGNORE_SELF 0.05//ロボット自身を意味する円オブジェクトは無視する

typedef struct{
	double x=0.0;
	double y=0.0;
}vector2D;

//3点の座標から角度を求める
double pointAngle(vector2D p1,vector2D p2,vector2D p3){
	vector2D ba;
	ba.x=p1.x-p2.x;
	ba.y=p1.y-p2.y;
	vector2D bc;
	bc.x=p3.x-p2.x;
	bc.y=p3.y-p2.y;
	double cross_product=ba.x*bc.y-ba.y*bc.x;//外積
	double dot_product=ba.x*bc.x+ba.y*bc.y;//内積
	double radian=atan2(cross_product,dot_product);
	double angle=radian/M_PI*180;
	return angle;
}

class ObstacleGet : public rclcpp::Node
{
public:
    ObstacleGet() : Node("req_sender")
    {
        setParam();
        // the state_suber is set to subscribe "/tracked_obstacles" topic
        obstacle_suber = this->create_subscription<obstacle_detector::msg::Obstacles>(
            "/tracked_obstacles", 10, std::bind(&ObstacleGet::obstacleCallback, this, _1));
        near_object_puber = this->create_publisher<std_msgs::msg::Int64>("/near_object", 10);
    };

private:
    void setParam(){
        this->declare_parameter<double>("segment_axis_x", 0.5);
        this->declare_parameter<double>("segment_axis_y", 0.5);
        this->declare_parameter<double>("circle_axis_x", 0.5);
        this->declare_parameter<double>("circle_axis_y", 0.5);
        this->declare_parameter<double>("detect_object", 2.0);
        this->declare_parameter<bool>("easy_log", false);

        this->get_parameter<double>("segment_axis_x", segment_axis_x_);
        this->get_parameter<double>("segment_axis_y", segment_axis_y_);
        this->get_parameter<double>("circle_axis_x", circle_axis_x_);
        this->get_parameter<double>("circle_axis_y", circle_axis_y_);
        this->get_parameter<double>("detect_object", detect_object_);
        this->get_parameter<bool>("easy_log", easy_log_);

        cout << "segment_axis_x:" << segment_axis_x_ << endl;
        cout << "segment_axis_y:" << segment_axis_y_ << endl;
        cout << "circle_axis_x:" << circle_axis_x_ << endl;
        cout << "circle_axis_y:" << circle_axis_y_ << endl;
        cout << "detect_object:" << detect_object_ << endl;
        cout << "easy_log:" << easy_log_ << endl;
    }

    void obstacleCallback(obstacle_detector::msg::Obstacles msg)
    {
        //壁オブジェクトに関する処理=========
        vector2D A_min,B_min;//最も近い壁の両端の座標
        vector2D C_min;//原点と最も近い壁オブジェクト上の点
        C_min.x = 1e+4;
        C_min.y = 1e+4;
        vector2D E_min;//LRFの真正面の座標

        int lr_flag=0;//左に壁があれば1,右なら-1
        double wd_min=1e+4;//最も近い壁を探す(wall d)
        double wall_size;
        double wall_angle;

        int status=0;//状態 0:障害物なし 1:近くに障害物あり 2:遠くに障害物あり
        std_msgs::msg::Int64 data;
	    std_msgs::msg::Float64MultiArray array;
        array.data.resize(3);//状態を伝える配列 0:状態 1:x座標 2:y座標
	    //intの識別情報をfloat64(c++だとdouble相当)で保存している。改善案を模索中

        for(int i=0,l=msg.segments.size();i<l;i++){
            vector2D A,B;//現在着目している壁オブジェクトの両端
            A.x=msg.segments[i].first_point.x, A.y=msg.segments[i].first_point.y;
            B.x=msg.segments[i].last_point.x, B.y=msg.segments[i].last_point.y;
            vector2D AB,AP;//ABベクトル,APベクトル(今回Pは観測者なので原点)
            AB.x=B.x-A.x;
            AB.y=B.y-A.y;
            AP.x=0-A.x;
            AP.y=0-A.y;

            vector2D C;//現在着目している原点と最も近い壁オブジェクト上の点
            double r=(AB.x*AP.x+AB.y*AP.y)/(AB.x*AB.x+AB.y*AB.y);//内積/|AB|
            double size=hypot(AB.x,AB.y);//壁の大きさ
            if(r<=0){//端の一つが最も近い点である
                C=A;
            }else if(r>=1){//端の一つが最も近い点である
                C=B;
            }else{//端以外の線分上に最も近い点である
                C.x=A.x+r*AB.x;
                C.y=A.y+r*AB.y;
            }
            double d=hypot(C.x,C.y);//求めた点と点Pとの距離

            if(A.x>0||B.x>0){//壁の両端が後ろのとき以外
                if(size>IGNORE_WALL){//小さすぎる壁は無視する
                    if(wd_min > d){
                        wd_min=d;
                        C_min=C;
                        wall_size=size;
                        if(r>0&&r<1){
                            A_min=A;
                            B_min=B;
                            lr_flag=(int)(C.y/fabs(C.y));//壁が近い場合左右どちらか判別する
                            double s=((A.x-0)*(B.y-A.y)-(A.y-0)*(B.x-A.x))/((1-0)*(B.y-A.y)-(0-0)*(B.x-A.x));
                            E_min.x=s;
                            E_min.y=0;
                        }
                    }
                }
            }
        }

        vector2D O;//原点(LRF位置)
        if(lr_flag<0){//LRF位置,LRFの真正面,壁の端の三点より壁の角度を求める
            wall_angle=pointAngle(O,E_min,A_min);
        }else{
            wall_angle=pointAngle(O,E_min,B_min);
        }
        if(E_min.x<0){//壁に対して背を向けている
            lr_flag=0;
        }
        //ここまで==============

        //円オブジェクトの処理================
        double cd_min=1e+4;//円オブジェクトで最も近いオブジェクト(circle)
        int object_id=0;
        int object_count=0;
        vector2D Ci_min;
        Ci_min.x = 1e+4;
        Ci_min.x = 1e+4;
        for(int i=0,l=msg.circles.size();i<l;i++){
            if(msg.circles[i].center.x > FRONT_LINE && hypot(msg.circles[i].center.x,msg.circles[i].center.y) > IGNORE_SELF){//オブジェクトが前方に存在 かつ　ロボット自身を捉えたものではない
                //(円の中心と原点までの距離)-(円の半径) は原点と円の位置関係を示す
                double d=hypot(msg.circles[i].center.x-0,msg.circles[i].center.y-0)-msg.circles[i].radius;
                if(cd_min > d){
                    cd_min = d;
                    object_id = i;
                    double l=hypot(msg.circles[i].center.x-0,msg.circles[i].center.y-0);
                    double r=msg.circles[i].radius;
                    Ci_min.x = ((l-r)/l)*msg.circles[i].center.x;
                    Ci_min.y = ((l-r)/l)*msg.circles[i].center.y;
                }
            }
            object_count++;
        }
        //ここまで============

        //最も近いオブジェクトが円か壁かを判断する
        if(wd_min < cd_min){
            if(ellipse(C_min, segment_axis_x_, segment_axis_y_)){
                data.data = 1;
                array.data[0]=1;
                array.data[1]=C_min.x;
                array.data[2]=C_min.y;
                if(debug_flag!=1){
                    cout << "####" << endl;
                    cout << "stop" << endl;
                    if(!easy_log_){
                        cout << "wd_wall:" << wd_min << endl;
                        cout << "C_min:{" << C_min.x << ", " << C_min.y << "}" << endl;
                        cout << "wall_size:" << wall_size << endl;
                        cout << "cd_min:" << cd_min << " object_count:" << object_count << endl; 
                    }
                    debug_flag=1;
                }
            }else{
                data.data = 0;
                array.data[0]=0;
                array.data[1]=1e+4;
                array.data[2]=1e+4;
                if(debug_flag!=0){
                    cout << "------" << endl;
                    cout << "no wall" << endl;
                    debug_flag=0;
                }
            }
        }else{
            if(ellipse(Ci_min, circle_axis_x_, circle_axis_y_)){
                data.data = 1;
                array.data[0]=1;
                array.data[1]=Ci_min.x;
                array.data[2]=Ci_min.y;
                if(debug_flag!=3){
                    cout << "#####" << endl;
                    cout << "stop" << endl;
                    if(!easy_log_){
                        cout << "circle[" << object_id << "]:" << cd_min << endl;
                        cout << "x:" << array.data[1] << " y:" << array.data[2] << " radius:" << msg.circles[object_id].radius << endl;
                        cout << "wd_wall:" << wd_min << endl;
                        cout << "cd_min:" << cd_min << " object_count:" << object_count << endl;
                    }
                    debug_flag=3;
                }
            }else{
                data.data = 0;
                array.data[0]=0;
                array.data[1]=1e+4;
                array.data[2]=1e+4;
                if(debug_flag!=0){
                    cout << "------" << endl;
                    cout << "no wall" << endl;
                    debug_flag=0;
                }
            }
        }
        near_object_puber->publish(data);
    }

    bool ellipse(vector2D P, double axis_x, double axis_y){
        double d = pow(P.x/axis_x, 2) + pow(P.y/axis_y, 2) - 1;
        if(d < 0){
            return true;
        }else{
            return false;
        }
    }

    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obstacle_suber;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr near_object_puber;

    int debug_flag=6;
    
    //パラメータ
    double detect_object_;
	bool easy_log_;
    double segment_axis_x_;
    double segment_axis_y_;
    double circle_axis_x_;
    double circle_axis_y_;
};

int main(int argc, char *argv[])
{
    cout << "start obstacle hit" << endl;

    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::spin(std::make_shared<ObstacleGet>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
