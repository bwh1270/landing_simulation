/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Woohyun Byun.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ********************************************************************************/
/**
 * @brief Basic Mathematics class 
 *
 * @author Woohyun Byun <imbwh@cau.ac.kr>
 */

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