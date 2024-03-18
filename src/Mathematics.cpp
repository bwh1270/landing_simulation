#include "landing_simulation/Mathematics.hpp"


Matrix4d calQuaternionMatrix(Vector4d q)
{
    // Assume: real part is q(0)
    Matrix4d qM;
    qM.resize(4,4);
    
    for (int i=0; i<4; i++) {
        qM(i,i) = q(0);
    }
    for (int i=0; i<3; i++) {
        qM(0,i+1) = -q(i+1);
        qM(i+1,0) = q(i+1);
    }
    qM(1,2) = -q(3);
    qM(2,1) = -qM(1,2);
    qM(1,3) = q(2);
    qM(3,1) = -qM(1,3);
    qM(2,3) = -q(1);
    qM(3,2) = q(1);
    
    return qM;
}

Vector3d quaternionToEuler(Vector4d& q, Vector3d& e)
{
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    e(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q(0) * q(2) - q(1) * q(3)));
    double cosp = std::sqrt(1 - 2 * (q(0) * q(2) - q(1) * q(3)));
    e(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    e(2) = std::atan2(siny_cosp, cosy_cosp);
}

Vector4d EulerToQuaternion(Vector3d& e, Vector4d& q)
{
    // roll (x), pitch (Y), yaw (z)
    // Abbreviations for the various angular functions
    double cr = cos(e(0) * 0.5);
    double sr = sin(e(0) * 0.5);
    double cp = cos(e(1) * 0.5);
    double sp = sin(e(1) * 0.5);
    double cy = cos(e(2) * 0.5);
    double sy = sin(e(2) * 0.5);

    q(0) = cr * cp * cy + sr * sp * sy;
    q(1) = sr * cp * cy - cr * sp * sy;
    q(2) = cr * sp * cy + sr * cp * sy;
    q(3) = cr * cp * sy - sr * sp * cy;
}

void quaternionToDCM(Vector4d& q, Matrix3d& DCM)
{
    Vector4d qq;
    for (int i=0; i<4; i++) {
        qq(i) = q(i)*q(i);
    }
    
    DCM(0,0) = qq(0)+qq(1)-qq(2)-qq(3);
    DCM(1,1) = qq(0)+qq(2)-qq(1)-qq(3);
    DCM(2,2) = qq(0)+qq(3)-qq(1)-qq(2);

    DCM(0,1) = 2*(q(1)*q(2)-q(0)*q(3));
    DCM(0,2) = 2*(q(1)*q(3)+q(0)*q(2));
    DCM(1,2) = 2*(q(2)*q(3)-q(0)*q(1));
    DCM(1,0) = 2*(q(1)*q(2)+q(0)*q(3));
    DCM(2,0) = 2*(q(1)*q(3)-q(0)*q(2));
    DCM(2,1) = 2*(q(2)*q(3)+q(0)*q(1));
}

void EulerToDCM(Vector3d& e, Matrix3d& DCM)
{
    // assume : euler angle is radian
    DCM(0,0) = cos(e(2)) * cos(e(1));
    DCM(1,0) = sin(e(2)) * cos(e(1));
    DCM(2,0) = -sin(e(2));

    DCM(1,0) = -sin(e(2))*cos(e(1)) + cos(e(2))*sin(e(1))*sin(e(0));
    DCM(1,1) = cos(e(2))*cos(e(0)) + sin(e(2))*sin(e(2))*sin(e(0));
    DCM(1,2) = cos(e(1)) * sin(e(0));

    DCM(2,0) = sin(e(2))*sin(e(0)) + cos(e(2))*sin(e(1))*cos(e(0));
    DCM(2,1) = -cos(e(2))*sin(e(0)) + sin(e(2))*sin(e(1))*cos(e(0));
    DCM(2,2) = cos(e(1)) * cos(e(0));
}

void setMatrixT(Matrix3d& DCM, Vector3d& t, Matrix4d& T)
{
    T = MatrixXd::Zero(4,4);
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            T(i,j) = DCM(i,j);
        }
    }

    for (int i=0; i<3; i++) {
        T(i,3) = t(i);
    }
    
    T(3,3) = 1;
}

void rotateVector(Matrix4d& T, Vector3d& ub, Vector3d& ua)
{
    Vector4d ua_h, ub_h;
    ub_h << ub(0), ub(1), ub(2), 1;
    ua_h = T*ub_h;
    for (int i=0; i<3; i++) {
        ua(i) = ua_h(i);
    }
}

void constrainXY(Vector2d* vxy, Vector2d* vff, float* max)
{
    assert( *max >= 0 );

    if (((*vxy) + (*vff)).norm() <= (*max)) {
        // vector does not exceed maximum magnitude
        *vxy = (*vxy) + (*vff);

    } else {
        *vxy = (*vxy).normalized() * (*max);
    }
}

void constrainZ(double* vz, float upMax, float downMax)
{
    assert(("upMax and downMax is opposite sign", downMax < 0));

    if (*vz >= upMax) {
        *vz = upMax;
    
    } else if (*vz <= downMax) {
        *vz = downMax;

    } else {
        return;
    }
}

