#ifndef CAL_POSITION_H
#define CAL_POSITION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
class Cal_Position
{
public:
  Cal_Position();
  void init(double currT);
  Vector3d getPosition(Matrix3d Cbn,Vector3d acc_b,double currT);
  float LowPassFilter_kalman(float data);
  double currT;
  Vector3d position;
  Vector3d vel_n;

  Vector3d acc_n;

};

#endif // CAL_POSITION_H
