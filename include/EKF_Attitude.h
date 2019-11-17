#ifndef EKF_ATTITUDE_H_
#define EKF_ATTITUDE_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "Convert.h"


#ifdef _WIN32
	#include "windows.h"
#else 
	#include <unistd.h>  
#endif

using namespace std;
namespace IMU
{

class EKF_Attitude
{
public:
    EKF_Attitude(bool approx_prediction, double dt);
    void init(Vector_12 init_measurement,double _currT);
    // Modify the params of the filter
    void Param_Change(Vector_12 Pro_Nosiecovr, Vector_9 Mea_Nosiecovr);

    // Calculate the transition matrix A
    // refer to formula 4.19a
    void Cal_TransMatrix();

    /* The priori prediction:
    *  Xk+1 = F(Xk,0)
    *  Pk+1 = A*Pk*A' + Qk
    */
    void Prior_Predict();

    // The posteriori correction
    void Post_Correct();

    // Calculate the Euler angular
    void Cal_Quaternion();

    // read the sensors
    void Read_SensorData(Vector_9 measurement);

    Eigen::Quaterniond Run(Vector_9 measurement,double _currT);
    void Release();
    void RequestStop();
    void RequestStart();
    bool Stop();
    bool isStopped();

    // Xk+1, the state vector of posteriori correction
   // vector< Vector_12 > state_X_pro;
    Vector_12 state_X_pro;
    double deltaT;

private:
    Vector_9 cur_measurement;
    bool using_2ndOrder;
    Eigen::Matrix<double, 12, 12> Q_noise;
    Eigen::Matrix<double, 9, 9> R_noise;

    Eigen::Matrix<double, 12, 12> Alin;



    // Xk, the state vector of priori prediction
    Vector_12 state_X_par;

    // Pk+1
    //vector< Eigen::Matrix<double, 12, 12> > CovrMatrix_P_pro;
    Eigen::Matrix<double, 12, 12> CovrMatrix_P_pro;

    // Pk
    Eigen::Matrix<double, 12, 12> CovrMatrix_P_par;
    Eigen::Quaterniond quaternion;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

} //namespace IMU

#endif
