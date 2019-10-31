#include <iostream>
#include <eigen3/Eigen/Dense>
#define pi 3.1415926f
using namespace  std;
int main()
{
  Eigen::AngleAxisd init_axis_x = Eigen::AngleAxisd(pi/2,Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd init_axis_y = Eigen::AngleAxisd(pi/4,Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd init_axis_z = Eigen::AngleAxisd(pi/3,Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d R = Eigen::Matrix3d(init_axis_z*init_axis_y*init_axis_x);
  Eigen::Vector3d euler = R.eulerAngles(2,1,0);
  cout << "Yaw : " << euler[0] << " Roll: " << euler[1] << " Pitch: " << euler[2] << endl;
  Eigen::Matrix<double,2,3> R_tilt;
  R_tilt << 1,0,0,0,1,0;
  cout << "TiltR:\n" << R_tilt << endl;
  Eigen::Vector2d tilt_angle = R_tilt*R*Eigen::Vector3d::UnitZ();
  cout << "Tilt angle : " << tilt_angle.transpose()*180/pi << endl;
}
