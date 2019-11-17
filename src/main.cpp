#include <iostream>
#include <ros/ros.h>
#include <fstream>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <tf/tf.h>

#include <deque>
#include <algorithm>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <cmath>

#include <EKF_Attitude.h>
#include <Convert.h>
#include <cal_position.h>

using namespace std;
using namespace Eigen;

struct imuData{
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Quaterniond pose;
};

struct gnssData{
    Eigen::Vector3d position;
};

struct magData{
    Eigen::Vector3d mag;
};

fstream file_laser("/home/msi/桌面/pose_data.csv",ios::ate|ios::out);

imuData imu_dev;
gnssData gps_dev;
magData mag_dev;
IMU::EKF_Attitude EKF_AHRS(true,0.001);
Cal_Position position;
ros::Publisher pose_pub;

ros::Publisher True_pose_pub;

ros::Publisher acc_n_pub;
//nav_msgs::Path path_msg;
//ros::Publisher pub_path;

//deque：双端队列，具有队列和栈的性质，可双端弹出，也有[]运算符重载
deque< pair<double, sensor_msgs::Imu> > imu_q;                          //imu
deque< pair<double, Vector3d> >fix_q;                                   //GPS
deque< pair<double, geometry_msgs::Vector3Stamped> >fix_velocity_q;     //GPS
deque< pair<double, sensor_msgs::MagneticField> > mag_q;             //磁力

bool processSensorData()
{
  //如果imu队列为空
  if(imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.15 ) return false;

  //计数器
  static int imu_cnt = 0;//correct with acc every 10 times
  //find the first com sensor
  double t[2] = {DBL_MAX};
  for(int i = 0; i < 6; i++) t[i] = DBL_MAX;

  //deque.front()返回第一个元素
  //pair.first,返回前面元素的值
  if(!imu_q.empty()) t[0] = imu_q.front().first;                    //取时间戳，赋值给t[0]
  if(!fix_q.empty()) t[1] = fix_q.front().first;

  //for(int i = 0; i < 6; i++) cout << i << " " << t[i] << "  ";

  //min_element：取容器最小值
  //min_id：时间最早的索引
  int min_id = min_element(t, t + 6) - t;
  //cout << "min_id: " << min_id << "  min_t: " << t[min_id] << endl;

  //如果时间最小值也是DBL_MAX，则函数直接返回false
  if(t[min_id] == DBL_MAX) return false;

  if(min_id == 0)//imu
  {
        //cout << "size: " << imu_q.size() << endl;
        double t = imu_q.front().first;                         //取时间戳
        sensor_msgs::Imu msg = imu_q.front().second;            //取imu数据内容
        Vector3d acc, gyro;                                     //定义加速度acc，角速度gyro
        acc(0) = msg.linear_acceleration.x;
        acc(1) = msg.linear_acceleration.y;
        acc(2) = msg.linear_acceleration.z;
        gyro(0) = msg.angular_velocity.x;
        gyro(1) = msg.angular_velocity.y;
        gyro(2) = msg.angular_velocity.z;
        //cout<<"LInear acc:"<<acc.transpose()<<endl;
        //cout<<"Ang gyro:"<<gyro.transpose()<<endl;
        //pose_ekf.predict(gyro, acc, t);                         //数据传入ekf，进行预测 （imu用来预测）
        imu_cnt++;                                              //imu预测计数+1
        //if(imu_cnt % 10 == 0) pose_ekf.correct_gravity(acc, t); //每10次使用correct_gravity修正（实际上是“更新”）
        imu_q.pop_front();                                      //弹出imu_q双端队列的第一个元素

    }else if(min_id == 1)//fix
    {

      double t = fix_q.front().first;
      Vector3d position = fix_q.front().second;                 //取GPS位置数据
      //pose_ekf.correct_fix(position, t);                        //更新ekf
      fix_q.pop_front();

    }/*else if(min_id == 5) //fix_velocity
    {

      double t = fix_velocity_q.front().first;
      geometry_msgs::Vector3Stamped msg = fix_velocity_q.front().second;    //取GPS速度数据
      Vector3d fix_velocity;
      fix_velocity(0) = msg.vector.x;
      fix_velocity(1) = msg.vector.y;
      fix_velocity(2) = msg.vector.z;
      pose_ekf.correct_fix_velocity(fix_velocity, t);           //更新ekf
      fix_velocity_q.pop_front();
    }*/
  return true;
}

//Vector3d align(Vector3d Gyro,Vector3d Acc,Vector3d Pos,Vector3d Ang){


//    double omg_N=WIE*cos(Pos[0]);
//    double omg_U=WIE*sin(Pos[0]);
//}

//INS ins;
bool first_imu_get_flag=false;
bool Align_flag=false;
double currT=0;
Vector3d align_pose;
Quaterniond curr_pose;
Quaterniond euler2quaternion(Vector3d euler)
{
  //euler:yaw,pitch,roll
  //qnb
  double cr = cos(euler(2)/2);
  double sr = sin(euler(2)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(0)/2);
  double sy = sin(euler(0)/2);
  Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q;
}
Quaterniond euler_312_To_quaternion(Vector3d euler)
{
  //euler:yaw,pitch,roll
  //qnb
  double cr = cos(euler(2)/2);
  double sr = sin(euler(2)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(0)/2);
  double sy = sin(euler(0)/2);
  Quaterniond q;
  q.w() = cy*cp*cr-sy*sp*sr;
  q.x() = cy*sp*cr-sy*cp*sr;
  q.y() = sy*sp*cr+cy*cp*sr;
  q.z() = sy*cp*cr+cy*sp*sr;
  return q;
}
Matrix3d C_nb_312_from_qnb(Quaterniond q){
  double q0=q.w();
  double q1=q.x();
  double q2=q.y();
  double q3=q.z();

  Matrix3d Cnb;
  Cnb<<q0*q0+q1*q1-q2*q2-q3*q3,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2),
      2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1),
      2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3;
  return Cnb.transpose();
}
Matrix3d C_nb_312_from_euler(Vector3d euler){
  double cy=cos(euler[0]);
  double sy=sin(euler[0]);

  double cp=cos(euler[1]);
  double sp=sin(euler[1]);

  double cr=cos(euler[2]);
  double sr=sin(euler[2]);

  Matrix3d Cnb_312;
  Cnb_312<<cr*cy-sp*sr*sy,cr*sy+cy*sp*sr,cp*sr,
            -cp*sy,cp*cy,-sp,
            -cy*sr-cr*sp*sy,cr*cy*sp-sr*sy,cp*cr;
  return Cnb_312;
}
Vector3d C_nb_312_to_euler(Matrix3d Cnb){
  Matrix3d Cbn=Cnb.transpose();
  double pitch,roll,yaw;
  pitch=asin(Cbn(2,1));
  if(fabs(Cbn(2,1))<=0.999999){
    roll=-atan2(Cbn(2,0),Cbn(2,2));
    yaw=-atan2(Cbn(0,1),Cbn(1,1));
  }else{
    roll=atan2(Cbn(0,2),Cbn(0,0));
    yaw=0;
  }
  return Vector3d(yaw,pitch,roll);
}
Matrix3d quaternion2matrix(Quaterniond q){
    double a=q.w();
    double b=q.x();
    double c=q.y();
    double d=q.z();

    Matrix3d result;
    //如果传入的q是目前的姿态，即qnb，是从参考系到载体坐标系的变换
    result<<a*a+b*b-c*c-d*d,   2*(b*c+a*d),    2*(b*d-a*c),
            2*(b*c-a*d)     ,   (a*a-b*b+c*c-d*d),  2*(c*d+a*b),
            2*(b*d+a*c)     ,   2*(c*d-a*b)     ,   (a*a-b*b-c*c+d*d);
    return result;
}
Vector3d matrix2euler(Matrix3d matrix){
  //使用Cbn计算
  Matrix3d DCM_Matrix=matrix.transpose();
  double pitch,roll,yaw;
  if ((DCM_Matrix(2,0) < -1)||(DCM_Matrix(2,0) > 1)) {
    //num_math_errors++;
  }
  else {
    pitch = -asin(DCM_Matrix(2,0)); // Attempt to prevent nan problems...
  }
  roll = atan2(DCM_Matrix(2,1),DCM_Matrix(2,2));
  yaw = atan2(DCM_Matrix(1,0),DCM_Matrix(0,0));
  return Vector3d(yaw,pitch,roll);
}
Vector3d getPoseFrom_Acc(Vector3d _acc){
    Vector3d acc=_acc;
    acc.normalize();
    double norm=sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
    Vector3d result;
    result<<0,atan2(acc[1],sqrt(acc[0]*acc[0]+acc[2]*acc[2])),-atan2(acc[0],acc[2]);
    return result;
}
Vector3d getYawFrom_Mag(Vector3d _mag,Vector3d pose){
    Vector3d mag=_mag;
    mag.normalize();
    double pitch=pose[1];
    double roll =pose[2];
    double mag_x=-mag[1]*cos(pitch)-mag[0]*sin(roll)*sin(pitch)+mag[2]*cos(roll)*sin(pitch);  // -M_n*cos(yaw)
    double mag_y=mag[0]*cos(roll)+mag[2]*sin(roll);  // M_n*sin(yaw)

    return Vector3d(atan2(mag_y,-mag_x),0,0);
}


void mahony(double currT){
    static double deltaT_prev=0;
    static bool init=false;
    if(!init){
        deltaT_prev=currT;
        init=true;
        return ;
    }
    Quaterniond q=curr_pose;//euler2quaternion(align_pose);

    //Quaterniond q=euler_312_To_quaternion(align_pose);
    //q.normalize();

    Matrix3d Cnb=C_nb_312_from_qnb(q);

    //Matrix3d Cnb=C_nb_312_from_euler(align_pose);
    Cnb.normalize();
    Vector3d pose=align_pose;
    Vector3d mag=mag_dev.mag;
    mag.normalize();
    Vector3d magRef=Cnb.transpose()*mag;  //得到n系下的磁力计分量

    Vector3d magRef_(0,sqrt(magRef[0]*magRef[0]+magRef[1]*magRef[1]),magRef[2]);

    //估计磁场方向
    Vector3d magDir_hat=0.5*Cnb*magRef_;

    //mag 与 方向进行叉积
    Vector3d mag_cross_magDir=mag.cross(magDir_hat);


    //累积
    Vector3d err_p(0,0,0);
    Vector3d err_i(0,0,0);

    double kp_yaw=1.2;
    double ki_yaw=0.0002;
    err_p=err_p+kp_yaw*mag_cross_magDir;
    err_i=err_i+ki_yaw*(currT-deltaT_prev)*mag_cross_magDir;

    //归一化加速度计输出
    Vector3d acc;
    acc=imu_dev.acc;
    acc.normalize();

    double q0=q.w();
    double q1=q.x();
    double q2=q.y();
    double q3=q.z();
    //Vector3d vv(q1*q3-q0*q2,q0*q1+q2*q3,q0*q0+q3*q3-0.5);//0.5*Cbn[2]
    Vector3d vv=Cnb*Vector3d(0,0,1);
    Vector3d acc_cross_vv=acc.cross(vv);

    double kp_rollpitch=1.2;
    double ki_rollpitch=0.0002;
    err_p=err_p+kp_rollpitch*acc_cross_vv;
    err_i=err_i+ki_rollpitch*(currT-deltaT_prev)*acc_cross_vv;

//    //cout<<acc.transpose()<<endl;

    //对陀螺仪输出的角速度进行修正
    Vector3d gyro=imu_dev.gyro;

    //gyro<<0.001,0.000,0.000;
    gyro=gyro+err_p+err_i;

    if((currT-deltaT_prev)>0){
        gyro=gyro*(currT-deltaT_prev);
    }
    else
        return;
    //cout<<gyro.transpose()<<endl;
    //通过修正后的角速度与原四元数计算四元数的导数
    Matrix4d gyro_mat;
    gyro_mat<<  0,    -gyro[0],   -gyro[1],   -gyro[2],
                gyro[0],    0,      gyro[2],    -gyro[1],
                gyro[1], -gyro[2],        0,    gyro[0],
                gyro[2],  gyro[1],  -gyro[0],   0;

    //Matrix4d q_diff=0.5*gyro_mat*q;
    double gyro_L2=sqrt(gyro[0]*gyro[0]+gyro[1]*gyro[1]+gyro[2]*gyro[2]);

    Vector4d q_curr;
    q_curr<<q.w(),q.x(),q.y(),q.z();
    Vector4d q_new=((1-gyro_L2*gyro_L2/8)*Matrix4d::Identity()+(0.5-gyro_L2*gyro_L2/48)*gyro_mat)*q_curr;


    //cout<<q_new.transpose()<<endl;

    Quaterniond output;
    output.w()=q_new[0];
    output.x()=q_new[1];
    output.y()=q_new[2];
    output.z()=q_new[3];
    output.normalize();
    curr_pose=output;
    deltaT_prev=currT;
    Matrix3d tmp=C_nb_312_from_qnb(output);
    tmp.normalize();
    align_pose=C_nb_312_to_euler(tmp);
    cout<<align_pose.transpose()*180/PI<<endl;

}

Quaterniond fast_Cal_Quaternion(){
    Vector3d acc_state,mag_state,angular_state;
    acc_state=imu_dev.acc;
    mag_state=mag_dev.mag;

    acc_state.normalize();
    mag_state.normalize();

    Vector3d x_state,y_state,z_state;

    z_state = acc_state;

    y_state = z_state.cross(mag_state);
    y_state.normalize();

    x_state = y_state.cross(z_state);
    x_state.normalize();

    // the rotation order is X-Y-Z,and
    Matrix3d Rotation_matrix;
    Rotation_matrix << x_state, y_state, z_state;
    Eigen::Quaterniond quat_temp(Rotation_matrix.transpose());
    return  quat_temp;
}
//pose发布
void publish_pose(Vector3d _pose)
{
  geometry_msgs::PoseStamped pose;
  //tf::Quaternion q=tf::createQuaternionFromRPY(_pose[2],-_pose[1],-_pose[0]+0.75*PI);
  Quaterniond q=euler_312_To_quaternion(_pose);
  //从ekf中获取修正数据
  //q.normalize();
  curr_pose=q;
  pose.header.stamp = ros::Time(currT);                     //融合时间戳
  pose.header.frame_id = "/world";
  pose.pose.orientation.w = q.w();                        //设置pose旋转参数
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.position.x = 1;                            //位置参数
  pose.pose.position.y = 0;
  pose.pose.position.z = 1;
  pose_pub.publish(pose);                                 //发布
}

void publish_pose(Quaterniond q)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time(currT);                     //融合时间戳
  pose.header.frame_id = "/world";
  pose.pose.orientation.w = q.w();                        //设置pose旋转参数
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.position.x = 1;                            //位置参数
  pose.pose.position.y = 0;
  pose.pose.position.z = 1;
  pose_pub.publish(pose);                                 //发布
}

bool init_Align()
{
  //如果imu队列为空
  if(imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.15 ) return false;

  //计数器
  static int imu_cnt = 0;//correct with acc every 10 times
  //find the first com sensor
  double t[2] = {DBL_MAX};
  for(int i = 0; i < 6; i++) t[i] = DBL_MAX;

  //deque.front()返回第一个元素
  //pair.first,返回前面元素的值
  if(!imu_q.empty()) t[0] = imu_q.front().first;                    //取时间戳，赋值给t[0]
  if(!fix_q.empty()) t[1] = fix_q.front().first;
  if(!mag_q.empty()) t[2] = mag_q.front().first;

  //min_element：取容器最小值
  //min_id：时间最早的索引
  int min_id = min_element(t, t + 6) - t;
  //cout << "min_id: " << min_id << "  min_t: " << t[min_id] << endl;

  //如果时间最小值也是DBL_MAX，则函数直接返回false
  if(t[min_id] == DBL_MAX) return false;

  if(min_id == 0)//imu
  {
      //cout << "size: " << imu_q.size() << endl;
      double t = imu_q.front().first;                         //取时间戳
      currT=t;
      sensor_msgs::Imu msg = imu_q.front().second;            //取imu数据内容
      Vector3d acc, gyro;                                     //定义加速度acc，角速度gyro
      acc(0) =  msg.linear_acceleration.x;
      acc(1) =  msg.linear_acceleration.y;
      acc(2) =  msg.linear_acceleration.z;
      gyro(0) = msg.angular_velocity.x;
      gyro(1) = msg.angular_velocity.y;
      gyro(2) = msg.angular_velocity.z;
      Quaterniond pose;
      pose.w()=msg.orientation.w;
      pose.x()=msg.orientation.x;
      pose.y()=msg.orientation.y;
      pose.z()=msg.orientation.z;
      //cout<<"LInear acc:"<<acc.transpose()<<endl;
      //cout<<"Ang gyro:"<<gyro.transpose()<<endl;
      imu_dev.acc=acc;
      imu_dev.gyro=gyro;
      imu_dev.pose=pose;



      static bool init=false;
      first_imu_get_flag=true;
      if(Align_flag){
          //mahony(currT);
          //publish_pose(curr_pose);

//          geometry_msgs::Vector3Stamped acc_n;
//          acc_n.vector.x=imu_dev.gyro[0];
//          acc_n.vect or.y=imu_dev.gyro[1];
//          acc_n.vector.z=imu_dev.gyro[2];
//          acc_n.header.stamp=ros::Time(currT);
//          acc_n.header.frame_id="world";
//          acc_n_pub.publish(acc_n);

          Eigen::VectorXd init_measurement(12);
          init_measurement<<imu_dev.gyro,0,0,0,imu_dev.acc,mag_dev.mag;

          if(!init){
              EKF_AHRS.init(init_measurement,currT);
              //position.init(currT);
              init=true;
              cout<<"init succ"<<endl;
              return false;
          }
          Eigen::VectorXd measurement(9);
          measurement<<imu_dev.gyro,imu_dev.acc,mag_dev.mag;

          Eigen::Quaterniond quaternion=EKF_AHRS.Run(measurement,currT);
          Vector3d gyro_corr=EKF_AHRS.state_X_pro.segment(0,3);
          Vector3d acc_corr=EKF_AHRS.state_X_pro.segment(6,3);
          Vector3d mag_corr=EKF_AHRS.state_X_pro.segment(9,3);

          geometry_msgs::Vector3Stamped acc_n;
          acc_n.vector.x=mag_corr[0];
          acc_n.vector.y=mag_corr[1];
          acc_n.vector.z=mag_corr[2];
          acc_n.header.stamp=ros::Time(currT);
          acc_n.header.frame_id="world";
          acc_n_pub.publish(acc_n);


          Vector3d pose_corr;
          pose_corr=getPoseFrom_Acc(acc_corr);
          pose_corr=pose_corr+getYawFrom_Mag(mag_corr,pose_corr);
          cout<<"Yaw-Pitch-Roll "<<pose_corr.transpose()*180/PI<<endl;
          Quaterniond output=euler_312_To_quaternion(pose_corr);
          publish_pose(pose_corr);
      }

      imu_cnt++;                                              //imu预测计数+1
      imu_q.pop_front();                                      //弹出imu_q双端队列的第一个元素

  }else if(min_id == 1)//fix
  {
      double t = fix_q.front().first;
      //currT=t;
      Vector3d position = fix_q.front().second;                 //取GPS位置数据

      gps_dev.position=position;

      fix_q.pop_front();

  }else if(min_id == 2)//magnetic
  {
      double t = mag_q.front().first;                           //取时间戳
      //currT=t;
      //geometry_msgs::Vector3Stamped msg = mag_q.front().second; //取磁力计数据内容
      sensor_msgs::MagneticField msg = mag_q.front().second; //取磁力计数据内容
      Vector3d mag;
      mag(0) = msg.magnetic_field.x;
      mag(1) = -msg.magnetic_field.y;
      mag(2) = -msg.magnetic_field.z;
      mag_dev.mag=mag;

      if(!Align_flag&&first_imu_get_flag){
          align_pose=getPoseFrom_Acc(imu_dev.acc);
          align_pose=align_pose+getYawFrom_Mag(mag,align_pose);
          cout<<"Yaw-Pitch-Roll"<<align_pose.transpose()*180/PI<<endl;

          //计算出来的Yaw-Roll-Pitch 是n系到b系变换
          //转换到余弦矩阵之后则是Cnb
          //align_pose[0]=align_pose[0]-0.5*PI/180;
          publish_pose(align_pose);

          //Matrix3d tmp=C_nb_312_from_qnb(curr_pose);
          //tmp.normalize();
          //cout<<C_nb_312_to_euler(tmp).transpose()*180/PI<<endl;

          Align_flag=true;
      }
      mag_q.pop_front();
  }
  return true;
}

void publishTruePose(Eigen::Quaterniond q,double t){
    geometry_msgs::PoseStamped pose;

    //从ekf中获取修正数据
    Eigen::Quaterniond _q=q;
    _q.normalize();

    Matrix3d CNB=quaternion2matrix(_q);
    CNB.normalize();
    Vector3d eular=matrix2euler(CNB);
    //Eigen::Quaterniond output=euler2quaternion(eular);
    //cout<<matrix2euler(CNB).transpose()*180/PI<<endl;
    //cout<<CNB<<endl;
    //curr_pose=q;
    pose.header.stamp = ros::Time(t);                     //融合时间戳
    pose.header.frame_id = "/world";
    pose.pose.orientation.w = _q.w();                        //设置pose旋转参数
    pose.pose.orientation.x = _q.x();
    pose.pose.orientation.y = _q.y();
    pose.pose.orientation.z = _q.z();
    pose.pose.position.x = 0;                            //位置参数
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    True_pose_pub.publish(pose);
}

//imu回调
void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    double t = imu_msg->header.stamp.toSec();

    Eigen::Quaterniond q;
    q.w()=imu_msg->orientation.w;
    q.x()=imu_msg->orientation.x;
    q.y()=imu_msg->orientation.y;
    q.z()=imu_msg->orientation.z;

    publishTruePose(q,t);

    imu_q.push_back(make_pair(t, *imu_msg) );     //push(时间戳，消息)
}

//GPS-位置回调
void fixCallback(const sensor_msgs::NavSatFixConstPtr & msg)
{

  static bool refInitialized = false;
  double t = msg->header.stamp.toSec();

  Eigen::Vector3d position;
  //todo, the data should be ENU, but it sames in NED? maybe a buf of hector quadrotor
  position(0) = msg->latitude * PI/180;
  position(1) = msg->longitude *PI/180;
  position(2) = msg->altitude ;
  // position(0) = locX;
  // position(1) = locY;
  // position(2) = locZ;
  //std::cout<<position.transpose()<<std::endl;
  fix_q.push_back(make_pair(t, position));
}

////GPS-速度回调
//void fixVelocityCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
//{
//    double t = msg->header.stamp.toSec();
//    fix_velocity_q.push_back(make_pair(t, *msg));
//}
//磁力计回调
//void magCallback(const geometry_msgs::Vector3StampedConstPtr &msg)
//{
//  double t = msg->header.stamp.toSec();
//  mag_q.push_back(make_pair(t, *msg));            //push(时间戳，消息)
//}

void magCallback(const sensor_msgs::MagneticFieldConstPtr &msg)
{
  double t = msg->header.stamp.toSec();
  mag_q.push_back(make_pair(t, *msg));            //push(时间戳，消息)
}

void truePoseCallback(const geometry_msgs::QuaternionStampedConstPtr &msg){
    //True_pose_pub
    geometry_msgs::PoseStamped pose;
    Quaterniond q;
    q.w()=msg->quaternion.w;
    q.x()=msg->quaternion.x;
    q.y()=msg->quaternion.y;
    q.z()=msg->quaternion.z;
    //从ekf中获取修正数据
    q.normalize();
    //curr_pose=q;
    msg->header.stamp.toSec();
    pose.header.stamp = ros::Time();                     //融合时间戳
    pose.header.frame_id = "/world";
    pose.pose.orientation.w = q.w();                        //设置pose旋转参数
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = 0;                            //位置参数
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    True_pose_pub.publish(pose);                                 //发布                                      //发布
}
int main (int argc, char **argv)
{
  //节点初始化
  ros::init(argc, argv, "integrated_nav");
  ros::NodeHandle nh;
  //路径发布器
  //pub_path = nh.advertise<nav_msgs::Path>("path", 10);
  //pose发布器
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_cal", 10);
  True_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/True_pose", 10);
  //path_msg.header.frame_id = "world";

  acc_n_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/acc_n", 10);

  ros::Subscriber sub_imu = nh.subscribe("/imu", 100, imuCallback);                           //订阅imu
  ros::Subscriber sub_fix = nh.subscribe("/fix", 100, fixCallback);                           //订阅GPS
  ros::Subscriber sub_mag = nh.subscribe("/mag", 100, magCallback);                //订阅磁力计
  //ros::Subscriber sub_fix_velocity = n.subscribe("/fix_velocity", 100, fixVelocityCallback); //订阅GPS-速度


  ros::Subscriber sub_orientation = nh.subscribe("/px4/orientation", 100, truePoseCallback);
  //循环
  ros::Rate loop_rate(50);          //设置循环频率为50Hz
  while(ros::ok()/*&&!Align_flag*/){
      ros::spinOnce();
      while(init_Align()){
      }
//      if(Align_flag){
//          publish_pose(align_pose);
//      }

      loop_rate.sleep();
  }
  cout<<"Align is OK!   "<<endl;
  cout<<"Yaw-Roll-Pitch "<<align_pose.transpose()*180/PI<<endl;
  cout<<endl;
//  ins.param(align_pose,gps_dev.position);


//  while(ros::ok()){
//      ros::spinOnce();
//      while(predict()){}
//      loop_rate.sleep();
//  }

//  while(ros::ok())
//  {
//    ros::spinOnce();

//    while(processSensorData()){}    //处理数据，直到全部数据处理完成？
//    //publish_pose(pose_ekf);         //发布

//    loop_rate.sleep();
//  }
  return 0;
}
