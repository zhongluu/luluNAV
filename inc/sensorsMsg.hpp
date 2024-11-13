/*
LULUNAV
Copyright (C) {{ 2024 }}  {{ yulu_zhong }}

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef _SNESORSMSG_
#define _SNESORSMSG_

#include <Eigen/Dense>

#define LIGHTSPEED 299792458 // m/s

using Eigen::Vector3d;

typedef struct tagSENSORMSG_IMUDATA
{
    long imuTime;
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
    long gnssTime;
    double utcTime;
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