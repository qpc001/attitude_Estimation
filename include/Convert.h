#ifndef CONVERT_H_
#define CONVERT_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//#include "tic_toc.h"
#define PI 3.1415926535898

using namespace std;
namespace IMU
{
typedef Eigen::Matrix<double, 2, 1> Vector_2;
typedef Eigen::Matrix<double, 3, 1> Vector_3;
typedef Eigen::Matrix<double, 4, 1> Vector_4;
typedef Eigen::Matrix<double, 6, 1> Vector_6;
typedef Eigen::Matrix<double, 7, 1> Vector_7;
typedef Eigen::Matrix<double, 9, 1> Vector_9;
typedef Eigen::Matrix<double, 12, 1> Vector_12;

typedef Eigen::Matrix3d Matrix_3;
const Matrix_3 Zeros_Matrix3 = Matrix_3::Zero(3, 3);
const Matrix_3 Ones_Matrix3 = Matrix_3::Ones(3, 3);
const Matrix_3 Identity_Matrix3 = Matrix_3::Identity(3, 3);

void Vect_to_SkewMat(Vector_3 Vector, Matrix_3 &Matrix);
void Angular_to_Mat(Vector_3 Vector, Eigen::Matrix<double, 4, 4> &Matrix);
//void Rotation_to_Qaut(Matrix_3 rotation, Eigen::Quaterniond &quat);
Vector_4 Quaternion_to_Vect(Eigen::Quaterniond q);
Eigen::Quaterniond QuatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Matrix_3 Quat_to_Matrix(Eigen::Quaterniond q);

// Eigen IO
Eigen::MatrixXd readFromfile(const string file);
bool writeTofile(Eigen::MatrixXd matrix, const string file);

// Eluer(Rotate vector) to rotation matrix
Matrix_3 Euler_to_RoatMat(Vector_3 Euler);

// Eluer(Rotate vector) to rotation matrix
Eigen::Quaterniond Euler_to_Quaternion(Vector_3 Euler);

// Quaternion to eulers
Vector_3 Quaternion_to_Euler(Eigen::Quaterniond q);

// Rotation matrix to eulers, the rotation order is X-Y-Z(roll,pitch and yaw)
Vector_3 Rotation_to_Euler(Matrix_3 Rotation);
// Rotation matrix to quaternion http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
Eigen::Quaterniond Rotation_to_Quater(Matrix_3 Rotation);

// Indirect Kalman Filter for 3D Attitude Estimation (Roumeliotis)
Eigen::Quaterniond BuildUpdateQuat(Eigen::Vector3d DeltaTheta);


} //namesapce IMU

#endif
