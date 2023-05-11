#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include<string>
using namespace std;

/// @brief 实现角度到弧度的转换
/// @param degree 
/// @return 
float Deg2Rad(float degree){
    return degree / 180.0 * acos(-1); 
}

/// @brief 返回一个向量或点旋转后的结果
/// @param Vec 
/// @param degree 
/// @return 
Eigen::Vector3f& VecRotate(Eigen::Vector3f &Vec, float degree){
    Eigen::Matrix3f Rotate;
    float rad = Deg2Rad(degree);
    Rotate << cos(rad), -sin(rad), 0, sin(rad), cos(rad), 0, 0, 0, 1;
    Vec = Rotate * Vec;
    return Vec;
}

/// @brief 返回一个点平移后的结果
/// @param Vec 
/// @param t_x 
/// @param t_y 
/// @return 
Eigen::Vector3f& VecTranslation(Eigen::Vector3f &Vec, float t_x, float t_y){
    Eigen::Matrix3f translation;
    translation << 1, 0, t_x, 0, 1, t_y, 0, 0, 1;
    Vec = translation * Vec;
    return Vec;
}

int main(){
    Eigen::Vector3f point(2.0f, 1.0f, 1.0f);
    cout << "Original Point: \n" << point << endl;
    cout << "Rotate 45: \n" << VecRotate(point, 45) << endl;
    // cout << sqrt(5) * cos(Deg2Rad(71.565f)) << endl;
    cout << "Translate -> (1, 2): \n" << VecTranslation(point, 1, 2) << endl;
    // cout << "Original Point: \n" << point << endl;
    system("pause");
}