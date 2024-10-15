//ウェイポイント走行プログラム(オブジェクトなし)
//240048 hashimoto
//2024.10.10~
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

using std::placeholders::_1;
using namespace std;//ネームスペースの宣言

//x,y座標の表現
typedef struct{
	double x=0.0;
	double y=0.0;
}vector2D;

//三点ABCの座標から角ABCを求める
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

//線分ABと点Pとの距離(符号付き)
double DPL(vector2D A,vector2D B,vector2D P){
	vector2D AB,AP;//ABベクトル,APベクトル
	AB.x=B.x-A.x;
	AB.y=B.y-A.y;
	AP.x=P.x-A.x;
	AP.y=P.y-A.y;

	double cross_product=AB.x*AP.y-AB.y*AP.x;//外積
	return cross_product/hypot(B.x-A.x,B.y-A.y);
}

//前の向いてた方向と今の向いていた方向の差(difference_orientation)を求める
double over180(double now,double pri){
	double diff=now-pri;

	while(diff >= 180){diff-=360;}
	while(diff < -180){diff+=360;}

	return diff;
}

//==================================================================================
//==================================================================================

//走行プログラム
class OdomTwist : public rclcpp::Node
{
public:
	//コンストラクタ
	OdomTwist() : Node("OdomTwist"){
		setParam();
		readFile();
		twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
		odom_sub_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10,std::bind(&OdomTwist::odomCallback,this,_1));
	}
	
private:
	//rosのpubsubに関する宣言
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr odom_sub_;

	//コールバック関数の宣言
	void odomCallback(const turtlesim::msg::Pose::SharedPtr odom_msg);

	//関数の宣言
	void setParam();
	void readFile();
	void step0();
	void step1();
	void step2();
	void step3();
	void step4();
	void step5();

	//停止命令(よく使うから関数にした)
	void stopMove(){
		twist.linear.x=0.0;
		twist.angular.z=0.0;
	}

	//位置情報のデバッグメッセージテンプレート
    	void posInfo(){
		printf("now_linear.x:%lf\n",now_pos.x);
		printf("now_linear.y:%lf\n",now_pos.y);
		printf("now_anguler.z:%lf\n",now_ori);
	}

	//速度がゼロになったときの位置を出力
	void speedOutput(){
		if(speed_debug_flag==0){
			if(now_speed_linear<0.1&&now_speed_angular<0.1){
				printf("------\n");
				printf("0 speed\n");
				posInfo();
				speed_debug_flag=1;
			}
		}
	}

	//ポテンシャル法
	double getOmega();
	double getPot(double, double);

	//------------------------------------------------------------
	//クラス内で共有する変数
	vector<vector2D> waypoint;//ロボット座標系ウェイポイント
	int WPC;//waypointの数(waypoint count)

	double angle_diff=0.0;
	double pri_ori=0;//直前角度
	double now_ori=0;//現在の角度

	int waypoint_flag=0;//どのウェイポイントまで通過したかを管理するフラグ
	int step_flag=0;//0で現在座標位置取得、1で前進、2で現在回転角度取得、3で回転 4更新 5ゴール 回転終了後にウェイポイント更新 9エラー
	vector2D set_pos;//ウェイポイント到達時の座標を格納(set_position)
	int debug_flag=0;//デバッグメッセージ用フラグ
	int mp_flag=0;//修正点(modify point)
	vector2D now_pos;//現在位置

	int speed_debug_flag=0;//スピードが0になったタイミングを知らせるデバッグフラグ
	int error_debug_flag=0;//エラー表示に関するデバッグフラグ
    int potential_flag=0;//ポテンシャル法用のフラグ

	double now_speed_linear=0.0;//現在の速度
	double now_speed_angular=0.0;//現在の角速度

	double total=0.0;//走行距離の合計

	geometry_msgs::msg::Twist twist;//速度命令の型の宣言
	
	//------------------------------------------------------------
	//パラメータ
	int mode_;
	string file_name_;
	double speed_;
	bool simple_log_;

    double weight_goal_;

	double goal_size_;
	double border_mp_;
	double stop_angle_;
};

//オドメトリデータを受け取ったときに呼び出される関数
void OdomTwist::odomCallback(const turtlesim::msg::Pose::SharedPtr odom_msg){
	now_pos.x=odom_msg->x;
	now_pos.y=odom_msg->y;
	now_ori=odom_msg->theta*180/M_PI;
	now_speed_linear=odom_msg->linear_velocity;
	now_speed_angular=odom_msg->angular_velocity;

	switch(step_flag){
        case 0:{
            step0();
        }break;
        case 1:{
            step1();
        }break;
        case 2:{
            step2();
        }break;
        case 3:{
            step3();
        }break;
        case 4:{
            step4();
        }break;
        case 5:{
            step5();
        }break;
        case 9:{
            if(error_debug_flag == 0){
                printf("--- error waypoint_a ---\n");
                error_debug_flag = 1;
            }
            stopMove();
        }break;
        default:{
            stopMove();
        }
    }

	twist_pub_->publish(twist);//速度命令を送る
}

//パラメータをセットする関数
void OdomTwist::setParam(){
	this->declare_parameter<int>("mode", 1);
	this->declare_parameter<string>("file_name", "sample.csv");
	this->declare_parameter<double>("speed", 0.1);

	this->get_parameter_or<int>("mode", mode_, 1);
	this->get_parameter_or<string>("file_name", file_name_, "sample.csv");
	this->get_parameter_or<double>("speed", speed_, 0.1);

	simple_log_ = false;
	weight_goal_ = 1.0;
	goal_size_ = 0.01;
	border_mp_ = 1.0;
	stop_angle_ = 1.0;

	cout << "mode:" << mode_ << endl;
	cout << "file_name:" << file_name_ << endl;
	cout << "speed:" << speed_ << endl;

    cout << "weight_goal:" << weight_goal_ << endl;
	if(weight_goal_ == 0.0){
		printf("~~~ debag mode ~~~\n");
		potential_flag=2;
	}

	cout << "goal_size:" << goal_size_ << endl;
	cout << "border_mp:" << border_mp_ << endl;

	if(speed_ > 1.0){
		cout << "speedが速すぎます"<< endl;
		cout << "speed は 1.0 m/s を指定してください" << endl;
		step_flag=9;
	}
}

//ウェイポイントファイルを読み込む関数
void OdomTwist::readFile(){
	if(file_name_ != "sample.csv"){
		mode_ = 2;
		cout << "ファイル名が入力されているため mode:2 で実行されます" << endl;
	}else if(mode_ == 1){
		cout << "mode:1 で実行されます" << endl;
	}else if(mode_ == 2){
		cout << "読み込みファイル名を sample.csv で実行します" << endl;
	}

	printf("use mode:%d\n",mode_);
	//ウェイポイントの設定
	if(mode_==1){
		//手打ちウェイポイントモード
		//double handpoint[][2]={{0,0},{25,0},{25,6.5},{-1.5,6.5},{-1.5,-3}};//ウェイポイントの座標(m):5号館前北向き
		double handpoint[][2]={{0,0},{1,0},{1,1},{0,0},{0.5,0}};//デバッグ
		int l=sizeof(handpoint)/sizeof(handpoint[0]);
		waypoint.resize(l);
		for(int i=0;i<l;i++){
			waypoint[i].x=handpoint[i][0];
			waypoint[i].y=handpoint[i][1];
		}
	}else if(mode_==2){
		//ファイル読み込みモード
		//スペースと改行コードからなるwaypoint.csvを読み込む.最終入力行は改行コード不要.
		char *user;
		if ((user = getlogin()) == NULL) perror("getlogin() error");
		string fileName1="/home/"+(string)user+"/colcon_ws/src/git-hashimoto/hashimoto_unitree/waypoint/"+file_name_;
		ifstream fs1;
		fs1.open(fileName1, ios::in);
		if(!fs1){
			cout<< file_name_ << "が開けないよ" << endl;
			cout << "ファイルは ~/colcon_ws/src/git-hashimoto/hashimoto_unitree/waypoint/ に置いてください" << endl;
		}else{
			cout<< fileName1 << endl;
			string line;
			while(getline(fs1,line)){
				vector<double> v;
				stringstream ss{line};
				string buf;
				while(getline(ss,buf,',')){
					v.push_back(stod(buf));
				}
				if(v.size()==2){
					vector2D v2;
					v2.x=v[0];
					v2.y=v[1];
					waypoint.push_back(v2);
				}else{
					cout<<"waypoint.csvは1行につきカンマで区切られた2要素で構成してください"<<endl;
				}
			}
		}
		fs1.close();
	}else{
		cout<<"modeは1(手打ち)か2(ファイル読み込み)を入力してください"<<endl;
	}

	//デバッグ用
	WPC=waypoint.size();
	if(WPC<2){
		cout << "ウェイポイントの生成に失敗しました" << endl;
		step_flag = 9;//エラー
	}else{
		for(int i=0;i<WPC;i++){
			printf("waypoint[%d]:{%lf,%lf}\n",i,waypoint[i].x,waypoint[i].y);
		}
		if(WPC>2){
			//3点の座標から外積を用いて回転すべき角度を求める.
			vector<double> angles;
			angles.resize(WPC-2);//コーナーの数(waypointの数-2)
			for(int i=0;i<WPC-2;i++){
				angles[i]=pointAngle(waypoint[i],waypoint[i+1],waypoint[i+2]);
				printf("angles[%d]:%lf\n",i,angles[i]);//0より大きい場合時計回りに角が存在(angular.z=-方向に回す)
			}
		}
	}
}

//スタート地点を保存
void OdomTwist::step0(){
	set_pos=now_pos;//ウェイポイントから移動開始の座標の初期位置を取得
	pri_ori=now_ori;//移動前の回転角を保存
	printf("======\n");
	printf("step_flag:%d,waypoint_flag:%d\n",step_flag,waypoint_flag);
	printf("lenear set\n");
	posInfo();//現在地,現在角度を出力
	step_flag=1;
	debug_flag=1;
}

//前進ステップ
void OdomTwist::step1(){
	double d2w=hypot(set_pos.x-now_pos.x,set_pos.y-now_pos.y);
	//distance traveled from waypoint 現在ウェイポイントからの移動距離
	double dbw=hypot(set_pos.x-waypoint[waypoint_flag+1].x,set_pos.y-waypoint[waypoint_flag+1].y);
	//distance between waypoint 現在ウェイポイントと次のウェイポイントの距離計算

	double dpl=DPL(set_pos,waypoint[waypoint_flag+1],now_pos);//distance from a point to a straight line 点と直線との距離
	double dnw=hypot(now_pos.x-waypoint[waypoint_flag+1].x,now_pos.y-waypoint[waypoint_flag+1].y);//distance now waypoint 現在地とウェイポイントとの距離


	if(debug_flag==1){
		printf("------\n");
		printf("step_flag:%d\n",step_flag);
		printf("linear\n");
		printf("dbw:%lf\n",dbw);
		//printf("dnw:%lf\n",dnw);
		printf("--\n");
		debug_flag=0;
	}

	//速度がゼロになったときの位置を出力
	if(!simple_log_){
		speedOutput();
	}

	//進行方向から一定以上離れたら修正を行う
	if(fabs(dpl)>border_mp_){
		stopMove();
		mp_flag=1;
		printf("------\n");
		printf("mp_flag:%d\n",mp_flag);
		printf("dpl:%lf\n",dpl);
		posInfo();
		step_flag=2;
	}

	if(dbw-d2w<0.0){
		//移動距離が次のウェイポイントまでの距離より大きいとき停止
		stopMove();//停止するまで待つ。
		if(now_speed_linear<0.1 && now_speed_angular<0.1){
			printf("------\n");
			printf("reach the waypoint type 1 dbw\n");
			posInfo();
			step_flag=2;
		}
		//すべてのウェイポイントを踏破した場合
		if(waypoint_flag==WPC-2){
			step_flag=5;//ゴールステップに移行
		}
	}else if(dnw<goal_size_){
		//ウェイポイントに十分近いとき(0.1m)も次のステップに移行する。
		stopMove();//停止するまで待つ。
		if(now_speed_linear<0.1 && now_speed_angular<0.1){
			printf("-----\n");
			printf("reach the waypoint type 2 dnw\n");
			posInfo();
			step_flag=2;
		}
		if(waypoint_flag==WPC-2){
			step_flag=5;//ゴールステップに移行
		}
	}else{
		//移動距離が次のウェイポイントまでの距離より小さい時前進
		//ポテンシャル法
		double omega=getOmega();

		twist.linear.x = speed_;
		twist.angular.z = omega*speed_;

		//(ポテンシャル法を一旦切りたい時)
		if(weight_goal_ == 0.0){
			twist.linear.x = speed_;
			twist.angular.z = 0.0;
		}else{//それ以外
			if(omega < -0.1){
				if(potential_flag==0||potential_flag==-1){
					printf("right:%lf\n",omega);
				}
				potential_flag=1;
			}else if(omega > 0.1){
				if(potential_flag==0||potential_flag==1){
					printf("left:%lf\n",omega);
				}
				potential_flag=-1;
			}else{
				if(potential_flag==1||potential_flag==-1){
					printf("front:%lf\n",omega);
				}
				potential_flag=0;
			}
		}
	}
}

//回転角計算
void OdomTwist::step2(){
	//絶対座標系の前方向からどのくらい回したところに次の点が存在するか
    vector2D front_pos;
    front_pos.x = now_pos.x + 1.0;
    front_pos.y = now_pos.y;
	double angle_set=pointAngle(front_pos,now_pos,waypoint[waypoint_flag+2-mp_flag]);
	//必要回転角を計算
	angle_diff=over180(angle_set,now_ori);//今向いている方向と目的の方向の差
    
	pri_ori=now_ori;
	printf("------\n");
	printf("step_flag:%d\n",step_flag);
	printf("angle set\n");
	if(!simple_log_){
		printf("next_waypoint[%d]{%lf,%lf}\n",waypoint_flag+2-mp_flag,waypoint[waypoint_flag+2-mp_flag].x,waypoint[waypoint_flag+2-mp_flag].y);
		//posInfo();
		printf("angle_set:%lf\n",angle_set);
	}
	printf("angle_diff:%lf\n",angle_diff);

	step_flag=3;
	debug_flag=1;
	speed_debug_flag=0;
}

//回転ステップ
void OdomTwist::step3(){
	//回転ステップ開始時からの回転量
	double diff_ori=over180(now_ori,pri_ori);

	if(debug_flag==1){
		printf("------\n");
		printf("step_flag:%d\n",step_flag);
		printf("angle\n");
		debug_flag=0;
	}

	//速度がおよそゼロになったときの位置を出力(センサーのゆらぎで0になることはない)
	if(!simple_log_){
		speedOutput();
	}

	//必要回転量と回転した角度の差がstop_angle_以上のとき回転
	if(fabs(angle_diff)-fabs(diff_ori) > stop_angle_){
		twist.linear.x=0.0;
		if(fabs(angle_diff)<0.01){//ほぼ180度ターンのときの回転方向
			twist.angular.z=speed_;
		}else{
			twist.angular.z=speed_*(angle_diff)/fabs(angle_diff);//(x/|x|)=1を使って回転方向を決定
		}
	}else{
		stopMove();
		if(now_speed_linear<0.1 && now_speed_angular<0.1){//止まるまで待つ
			printf("rotate:%lf\n",fabs(diff_ori));
			step_flag=4;
		}
	}
}

//フラグ更新ステップ
void OdomTwist::step4(){
	waypoint_flag=waypoint_flag-mp_flag+1;
	printf("step_flag:%d\n",step_flag);
	printf("mp_flag:%d\n",mp_flag);
	printf("next_waypoint[%d]{%lf,%lf}\n",waypoint_flag+1,waypoint[waypoint_flag+1].x,waypoint[waypoint_flag+1].y);
	if(mp_flag==0){
		total+=hypot(waypoint[waypoint_flag].x-waypoint[waypoint_flag-1].x,waypoint[waypoint_flag].y-waypoint[waypoint_flag-1].y);
		printf("total:%lf\n",total);
		printf("waypoint chenged\n");
		printf("******\n");
	}
	mp_flag=0;
	step_flag=0;
	debug_flag=0;
	potential_flag=0;
	speed_debug_flag=0;
}

//ゴールステップ
void OdomTwist::step5(){
	stopMove();
	if(debug_flag==0){
		printf("------\n");
		printf("goal\n");
		printf("(^o^)/\n");
		total+=hypot(waypoint[waypoint_flag+1].x-waypoint[waypoint_flag].x,waypoint[waypoint_flag+1].y-waypoint[waypoint_flag].y);
		printf("total:%lf\n",total);
		posInfo();
		debug_flag=1;
	}
}

//ポテンシャル法から角速度を計算する
double OdomTwist::getOmega(){
	double delta = 0.1;
	double vx,vy = 0.0;
	vx = -(getPot(now_pos.x+delta, now_pos.y) - getPot(now_pos.x, now_pos.y)) / delta;
	vy = -(getPot(now_pos.x, now_pos.y+delta) - getPot(now_pos.x, now_pos.y)) / delta;
	double v=hypot(vx, vy);
	vx /= v/(speed_);
	vy /= v/(speed_);

	double omega = over180(atan2(vy , vx)*180/M_PI,now_ori);
	//cout << "omega: " << omega << endl;
	omega=omega/180*M_PI;
	
	return omega;
}

//ポテンシャル場の高さを計算する
double OdomTwist::getPot(double px, double py){
	double goal_pot = -1.0 / hypot(px-waypoint[waypoint_flag+1].x, py-waypoint[waypoint_flag+1].y);
	double pot = goal_pot * weight_goal_;
	return pot;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	cout << "start" << endl;
	sleep(1);//走り出すまでの待ち時間
	auto node = std::make_shared<OdomTwist>();
	rclcpp::spin(node);

	rclcpp::shutdown();
    return 0;
}
