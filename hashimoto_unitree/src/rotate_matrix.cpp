//斜めについているlivox/lidarを回転させて平坦につけたときの座標に変換する
//hashimoto240048@gmail.com
//2024.10.23~
//回転行列計算プログラム
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;
using namespace std;

int main()
{
    Matrix3d m;
    m << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    
    double r = -1.0 * M_PI / 180;
    double p = -15.0 * M_PI / 180;
    double y = 0.0 * M_PI / 180;

    Matrix3d m_r;
    m_r << 1, 0, 0,
            0, cos(r), sin(r),
            0, -sin(r), cos(r);
    
    Matrix3d m_p;
    m_p << cos(p), 0, -sin(p),
            0, 1, 0,
            sin(p), 0, cos(p);

    Matrix3d m_y;
    m_y << cos(y), sin(y), 0,
            -sin(y), cos(y), 0,
            0, 0, 1;

    Matrix3d m_rpy;
    m_rpy = m * m_r * m_p * m_y;

    cout << m << endl;

    //std::cout << m_p << std::endl;
    //std::cout << m_rpy << std::endl;
    cout << fixed;
    for(int i = 0; i < 3; i++ ){
        for(int j = 0; j < 3; j++ ){
            cout << setprecision(3) << m_rpy(i, j) << ", "; 
        }
        cout << "\n";
    }
    cout << endl;
}