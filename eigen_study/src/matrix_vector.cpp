#include <iostream>
#include <eigen3/Eigen/Dense>
#define pi 3.1415926f
using namespace  std;

int main()
{
Eigen::Matrix<double,3,4> acc_paras;
acc_paras << 1,2,3,4,1,2,3,4,1,2,3,4;
Eigen::Vector4d info(1,2,3,1);
Eigen::Vector3d result = acc_paras*info;
//result = acc_paras*info;
cout << result << endl;
cout << info << endl;
}
