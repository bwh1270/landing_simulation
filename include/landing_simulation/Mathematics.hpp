#ifndef __MATHEMATICS__
#define __MATHEMATICS__

#include <eigen3/Eigen/Dense>
using namespace Eigen;

// calculate the quaternion matrix [q]
Matrix4d calQuaternionMatrix(Vector4d q);

// convert Quaternion to Euler Angles
Vector3d convertQuaternionToEuler(Vector4d q);

// convert Euler Angles to Quaternion
Vector4d convertEulerToQuaternion(Vector3d euler);

// control input + feedforward term with Saturation to Max Horizontal Velocity
void constrainXY(Vector2d* v, Vector2d* vff, float* max);

// Saturation to Vertical velocity 
void constrainZ(double* vec, float upMax, float downMax);


                    
#endif