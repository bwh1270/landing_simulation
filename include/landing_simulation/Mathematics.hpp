#ifndef __MATHEMATICS__
#define __MATHEMATICS__

#include <iostream>
#include <cassert>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

// calculate the quaternion matrix [q]
Matrix4d calQuaternionMatrix(Vector4d q);

// convert Quaternion to Euler Angles
Vector3d quaternionToEuler(Vector4d& q, Vector3d& e);

// convert Euler Angles to Quaternion
Vector4d EulerToQuaternion(Vector3d& e, Vector4d& q);

// convert Quaternion to Direction Cosine Matrix
void quaternionToDCM(Vector4d& q, Matrix3d& DCM);

// convert Euler angles to DCM
void EulerToDCM(Vector3d& euler, Matrix3d& DCM);

// set transformation matrix with DCM and translate vector
void setMatrixT(Matrix3d& DCM, Vector3d& t, Matrix4d& T);

// rotate the vector in {b} to {a}
void rotateVector(Matrix4d& T, Vector3d& ub, Vector3d& ua);

// control input + feedforward term with Saturation to Max Horizontal Velocity
void constrainXY(Vector2d* v, Vector2d* vff, float* max);

// Saturation to Vertical velocity 
void constrainZ(double* vec, float upMax, float downMax);


                    
#endif