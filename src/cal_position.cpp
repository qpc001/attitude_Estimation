#include "cal_position.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
using namespace std;
using namespace Eigen;
Cal_Position::Cal_Position()
{

}

void Cal_Position::init(double _currT){
  currT=_currT;
  vel_n<<0,0,0;
  position<<0,0,0;
}

float Cal_Position::LowPassFilter_kalman(float data)
{
  static float Xlast=0.00;//初值 Xlast&Xpre
  static float P=0.5;//Plast&Pnow
  const float Q=1;//自己感觉的方差
  const float R=1;//测量器件的方差
  float Kg,PP;//
  float Xnow;//经过卡尔曼滤波的值 Xnow
  //1式A=1 无输入
  PP=P+Q; //Ppre=Plast+Q （2）
  Kg=PP/(PP+R); //更新Kg Ppre/Ppre+R（4）
  Xnow=Xlast+Kg*(data-Xlast); // Xnow = Xpre+Kg*(Z(k)-H*Xpre)(3)
  P=(1-Kg)*PP; //Pnow=(I-Kg)*Ppre(5) 由于Pnow不会再使用，所以更新Plast=Pnow
  Xlast=Xnow;

  return Xnow;
}

Vector3d Cal_Position::getPosition(Matrix3d Cbn, Vector3d acc_b, double _currT)
{
  double deltaT=_currT-currT;
  acc_n=Cbn*acc_b;
  Vector3d g(0,0,9.636);
  //cout<<deltaT<<endl;
  //cout<<acc_n.transpose()<<endl;
  acc_n=acc_n-g;


  acc_n[0]=fabs(acc_n[0])>0.5? acc_n[0]:0;
//  acc_n[1]=fabs(acc_n[1])>0.1? acc_n[1]:0;
//  acc_n[2]=fabs(acc_n[2])>0.1? acc_n[2]:0;
  acc_n[0]=LowPassFilter_kalman(acc_n[0]);
  cout<<acc_n.transpose()<<endl;
  vel_n=vel_n+deltaT*(acc_n);

  position=position+deltaT*vel_n+0.5*deltaT*deltaT*acc_n;
  currT=_currT;
  return position;
}
