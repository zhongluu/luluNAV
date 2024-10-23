#ifndef _SNESORSMSG_
#define _SNESORSMSG_

#include <Eigen/Dense>

#define LIGHTSPEED 299792458 // m/s

using Eigen::Vector3d;

typedef struct tagSENSORMSG_IMUDATA
{
    Vector3d gyro; // x y z
    Vector3d acc; // x y z
}SENSORMSG_IMUDATA;

typedef struct tagSENSORMSG_SATDATA
{
    bool isValid;
    float satClk = 0.0; // (m)
    float lonoDly = 0.0; // (m)
    float tropDly = 0.0; // (m)
    Vector3d satECEF; // x y z
}SENSORMSG_SATDATA;


typedef struct tagSENSORMSG_GNSSDATA
{
    uint16_t prn = 0;
    uint16_t psrstd = 0;
    uint16_t CN0 = 0;
    uint32_t type = 0;
    uint32_t freqType = 0;
    double psr;
    double psrRate;
    float satClk = 0.0; // (m)
    float lonoDly = 0.0; // (m)
    float tropDly = 0.0; // (m)
    Vector3d pos;
    Vector3d satECEF; // x y z
}SENSORMSG_GNSSDATA;


#endif