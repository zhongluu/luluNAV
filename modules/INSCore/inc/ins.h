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
