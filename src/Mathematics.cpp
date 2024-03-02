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
    qM(3,1) = -qM(3,1);
    qM(2,3) = -q(1);
    qM(3,2) = q(1);
    
    return qM;
}


Vector3d convertQuaternionToEuler(Vector4d q)
{
    Vector3d euler;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q(0) * q(2) - q(1) * q(3)));
    double cosp = std::sqrt(1 - 2 * (q(0) * q(2) - q(1) * q(3)));
    euler(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

Vector4d convertEulerToQuaternion(Vector3d euler) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions
    double cr = cos(euler(0) * 0.5);
    double sr = sin(euler(0) * 0.5);
    double cp = cos(euler(1) * 0.5);
    double sp = sin(euler(1) * 0.5);
    double cy = cos(euler(2) * 0.5);
    double sy = sin(euler(2) * 0.5);

    Vector4d q;
    q(0) = cr * cp * cy + sr * sp * sy;
    q(1) = sr * cp * cy - cr * sp * sy;
    q(2) = cr * sp * cy + sr * cp * sy;
    q(3) = cr * cp * sy - sr * sp * cy;

    return q;
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

