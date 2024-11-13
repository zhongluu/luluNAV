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

#ifndef _INS_
#define _INS_

#include <iostream>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include "BASEElement.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::Matrix;

class INS
{
private:
    /* data */
public:

    Matrix<double, 3, 3> Cnb, Kg, Ka, Cw, MpvCnb;
    Vector3d  atti, vn, pos, biasGyro, biasAcc, lever; // atti - yaw pitch roll
    Vector3d estVn, estPos;
    Quaterniond qnb;
    double initialBiasG, initialBiasA, rNHC;
    double ts, deviationOfGyroV, deviationOfAccV, deviationOfAccU, dt;
    Vector3d  wib, wnb, fb, fn, an, web;

    Vector3d  deltaGyroPre, deltaAccPre, phim, dvbm, Mpvvn;
    Earth eth;
    std::string outputPath;
    std::fstream outfile;
    virtual bool imuCompensate(Vector3d &deltaGyro, Vector3d &deltaAcc) = 0; 
    virtual bool insUpdate(Vector3d& deltaGyro, Vector3d& deltaAcc) = 0; 
    virtual bool insConfig(const std::string& configFilePath) = 0; 
    virtual bool insOutput() = 0; 
    virtual bool insLogFile() = 0; 
};


class INSCreator
{
private:
    /* data */
public:
    virtual INS* CreateINS() = 0; 
};



#endif
