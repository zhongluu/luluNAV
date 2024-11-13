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

#include "insTC.h"
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <gsl/gsl_sf_psi.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaterniond;
using Eigen::Matrix;

TCINS::TCINS(/* args */):isFirstIMUData(true), isFileOpen(false)
{
}

TCINS::~TCINS()
{
}

bool TCINS::imuCompensate(Vector3d &deltaGyro, Vector3d &deltaAcc)
{
    if (isFirstIMUData) {
        deltaGyroPre = deltaGyro;
        deltaAccPre = deltaAcc;
        isFirstIMUData = false;
    }
    // coning compensate
    phim = deltaGyro +  (1.0 / 12.0) * deltaGyroPre.cross(deltaGyro);
    // sculling compensate
    dvbm = deltaAcc + 0.5 * (deltaGyro.cross(deltaAcc)) \
        + (1.0 / 12.0) * deltaGyroPre.cross(deltaAcc) + (1.0 / 12.0) \
        * deltaAccPre.cross(deltaGyro);
    deltaGyroPre = deltaGyro;
    deltaAccPre = deltaAcc;
    return true;
}

bool TCINS::insConfig(const std::string& configFilePath)
{
    std::map<std::string, std::string> config;
    std::ifstream file(configFilePath);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << configFilePath << std::endl;
    }
    
    while (std::getline(file,line)) {
        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            config[key] = value;
        }
        
    }

    file.close();
    std::string insmode = "TCINS";
    if (config["insmode"] != insmode) {
        std::cout << "configration mode is not tightly couple mode" <<std::endl;
        return false;
    }

    double pitch0 = stod(config["pitch0"]);
    double roll0 = stod(config["roll0"]);
    double yaw0 = stod(config["yaw0"]);
    double vn0x = stod(config["vn0x"]);
    double vn0y = stod(config["vn0y"]);
    double vn0z = stod(config["vn0z"]);
    double latitude0 = stod(config["latitude0"]);
    double longitude0 = stod(config["longitude0"]);
    double height0 = stod(config["height0"]);
    double gyrobias0x = stod(config["gyrobias0x"]);
    double gyrobias0y = stod(config["gyrobias0y"]);
    double gyrobias0z = stod(config["gyrobias0z"]);
    double accbias0x = stod(config["accbias0x"]);
    double accbias0y = stod(config["accbias0y"]);
    double accbias0z = stod(config["accbias0z"]);
    double lever0x = stod(config["lever0x"]);
    double lever0y = stod(config["lever0y"]);
    double lever0z = stod(config["lever0z"]);
    dt = stod(config["dt"]);
    double clockcorrection0 = stod(config["clockcorrection0"]);
    double clockdrift0 = stod(config["clockdrift0"]);
    double imufs = stod(config["imufs"]);
    rNHC = stod(config["NHCR"]);
    initialBiasG = stod(config["initialBiasG"]);
    initialBiasA = stod(config["initialBiasA"]);
    clock_phase_PSD = stod(config["clock_phase_PSD"]); 
    clock_freq_PSD = stod(config["clock_freq_PSD"]); 
    pseudo_range_SD = stod(config["pseudo_range_SD"]); 
    range_rate_SD = stod(config["range_rate_SD"]); 
    deviationOfGyroV = stod(config["devGyroV"]); 
    deviationOfAccV = stod(config["devAccV"]); 
    deviationOfAccU = stod(config["devAccU"]); 
    rx_clock_offset = stod(config["rx_clock_offset"]);
    rx_clock_drift = stod(config["rx_clock_drift"]);
    outputPath = config["outputpath"];
    atti << pitch0 * navGlv::deg2rad, roll0 * navGlv::deg2rad, yaw0 * navGlv::deg2rad;
    Eigen::AngleAxisd pitchAngle(atti[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rollAngle(atti[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(atti[2], Eigen::Vector3d::UnitZ());
    qnb =  yawAngle *pitchAngle* rollAngle  ;
    Cnb = qnb.toRotationMatrix(); 
    vn << vn0x, vn0y, vn0z;
    pos << latitude0, longitude0, height0;
    biasGyro << gyrobias0x, gyrobias0y, gyrobias0z;
    biasAcc << accbias0x, accbias0y, accbias0z;
    lever << lever0x, lever0y, lever0z;
    Kg.setIdentity();
    Ka.setIdentity();
    ccorr = clockcorrection0;
    cdrift =  clockdrift0;
    ts = 1.0 / imufs;
    
    estVn.setZero();
    estPos.setZero();
    return true;
}

bool TCINS::insLogFile()
{
    if (!isFileOpen)
    {
        outfile.open(outputPath, std::ios::out);
        if (!outfile.is_open()) {
            std::cerr << "Failed to open config file: " << outputPath << std::endl;
        }else{
            isFileOpen = true;
        }
    }
    
    return true;
    
}

bool TCINS::insOutput()
{
    return true;
}

bool TCINS::insUpdate(Vector3d& deltaGyro, Vector3d& deltaAcc)
{
    imuCompensate(deltaGyro, deltaAcc); 
    phim = Kg * phim - biasGyro * ts;  
    dvbm = Ka * dvbm - biasAcc * ts; 
    tmpVn = vn + 0.5 * an * ts; 
    eth.vnUpdateDpos(tmpVn); 
    tmpPos = pos + 0.5 * eth.dpos * ts; 
    eth.update(tmpPos, tmpVn);
    wib = phim / ts; fb = dvbm / ts; 
    web = wib - Cnb.transpose() * eth.wnie; 
    wnb = wib - Cnb.transpose() * eth.wnin; 
    fn = qnb * fb; 
    Vector3d tmpVec = -eth.wnin * ts;
    double angle = tmpVec.norm();
    tmpVec.normalize();
    Eigen::AngleAxisd phiwnin(angle, tmpVec);
    an = Eigen::Quaterniond(phiwnin) * fn + eth.gcc; 
    // velocity update 
    tmpVn = vn + an * ts; 
    tmpVec = tmpVn + vn;
    eth.vnUpdateDpos(tmpVec); 
    Mpvvn = ((0.5 * (eth.Mpv)) * (tmpVec));
    vn =  tmpVn;
    // pos update 
    pos = pos + eth.dpos * ts; 
    // attitude update 
    angle = phim.norm();
    phim.normalize();
    Eigen::AngleAxisd phi(angle, phim);
    qnb = Eigen::Quaterniond(phiwnin) * qnb * Eigen::Quaterniond(phi); 
    Cnb = qnb.toRotationMatrix(); 
    atti << std::asin(Cnb(2,1)), 
            std::atan2(Cnb(2,0),Cnb(2,2)),
            std::atan2(-Cnb(0,1),Cnb(1,1));
    if (Cnb(2,1) > 0.999999) {
        atti(1) = 0;
        atti(1) = std::atan2(Cnb(0,2),Cnb(0,0));
    } else if(Cnb(2,1) < -0.999999) {
        atti(1) = 0;
        atti(1) = -std::atan2(Cnb(0,2),Cnb(0,0));
    }
    // clock drift update
    ccorr = ccorr + cdrift * ts;
    return true;
}

bool TCINS::setPos(Vector3d &gnssPos)
{
    pos = gnssPos;
    return true;
}

bool TCINS::xyz2blh(Vector3d& xyz, Vector3d& blh)
{
    return true;
}

bool TCINS::getECEFFromSats(double& psr, Vector3d& deltaGyro, Vector3d&xECEF)
{
    return true;
}

TCINSSTATE TCINS::getInsState() const
{
    return {atti, vn, pos, biasGyro, biasAcc, lever, dt, ccorr, cdrift};
}

bool TCINS::seqEKF::init()
{
    double varOfGyroV = ins.deviationOfGyroV * ins.deviationOfGyroV;
    double varOfAccV = ins.deviationOfAccV * ins.deviationOfAccV;
    double varOfAccU = ins.deviationOfAccU * ins.deviationOfAccU;
    double varclock_freq_PSD = ins.clock_freq_PSD * ins.clock_freq_PSD;
    double varclock_phase_PSD = ins.clock_phase_PSD * ins.clock_phase_PSD;
    Q(0,0) = varOfGyroV; Q(1,1) = varOfGyroV; Q(2,2) = varOfGyroV; 
    Q(3,3) = varOfAccV; Q(4,4) = varOfAccV; Q(5,5) = varOfAccV;
    Q(12,12) = varOfAccU; Q(13,13) = varOfAccU; Q(14,14) = varOfAccU;
    Q(19,19) = varclock_phase_PSD; Q(20,20)= varclock_freq_PSD;
    P(0,0) = ((10.0/3.0)*navGlv::deg2rad)*((10.0/3.0)*navGlv::deg2rad);
    P(1,1) = P(0,0); P(2,2) = P(0,0);
    P(3,3) = (0.15/3)*(0.15/3);
    P(4,4) = P(3,3); P(5,5) = P(3,3);
    P(6,6) = 1e-6*1e-6; P(7,7) = P(6,6); P(8,8) = (20.0/3.0) * (20.0/3.0);
    P(9,9) = ins.initialBiasG * navGlv::dph * ins.initialBiasG * navGlv::dph;
    P(10,10) = P(9,9);
    P(11,11) = P(9,9);
    P(12,12) = ins.initialBiasA * ins.initialBiasA; 
    P(13,13) = P(12,12); 
    P(14,14) = P(12,12); 
    P(15,15) = ins.lever[0] * ins.lever[0]; 
    P(16,16) = ins.lever[1] * ins.lever[1]; 
    P(17,17) = ins.lever[2] * ins.lever[2]; 
    P(18,18) = ins.dt * ins.dt;
    P(19,19) = ins.rx_clock_offset * ins.rx_clock_offset;
    P(20,20) = ins.rx_clock_drift * ins.rx_clock_drift;
    R = ins.pseudo_range_SD * ins.pseudo_range_SD;
    R2(0,0) = ins.pseudo_range_SD * ins.pseudo_range_SD;
    R2(1,1) = ins.range_rate_SD * ins.range_rate_SD;
    NHCR.setZero();
    NHCR(0,0) = ins.rNHC * ins.rNHC;
    NHCR(1,1) = NHCR(0,0);
    return true;
}

bool TCINS::seqEKF::JacobianFt()
{
    double tl = ins.eth.tl, secl = 1.0 / ins.eth.cl;
    double secl2 = secl * secl, wN = ins.eth.wnie[1], 
            wU = ins.eth.wnie[2], vE = ins.vn[0], vN = ins.vn[1];
	double f_RMh = ins.eth.f_RMh, f_RNh = ins.eth.f_RNh, f_clRNh = ins.eth.f_clRNh; 
	double f_RMh2 = f_RMh * f_RMh, f_RNh2 = f_RNh * f_RNh;
    Matrix3d Avn = vector2skew(ins.vn);
    Mp1 <<  0,      0,       0,
           -wU,     0,       0,
            wN,     0,       0;
    Mp2 <<            0,          0,      vN * f_RMh2,
                      0,          0,     -vE * f_RNh2,
           vE * secl2 *  f_RNh,   0,   -vE * tl * f_RNh2;
    Vector3d tmpVec = -ins.eth.wnin;
    Maa = vector2skew(tmpVec);
    Mav <<       0,       -f_RMh,    0, 
               f_RNh,        0,      0, 
            tl * f_RNh,      0,      0;
    Map = Mp1 + Mp2;
    Mva = vector2skew(ins.fn);
    tmpVec = ins.eth.wnie + ins.eth.wnin;
    Mvv = Avn * Mav - vector2skew(tmpVec);
    Mvp = Avn * (Mp1 + Map);
    double scl = ins.eth.sl*ins.eth.cl;
    Mvp(2,0) = Mvp(2,0) - navGlv::g * (5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * ins.eth.sl2 * scl);
    Mvp(2,2) = Mvp(2,2) + 3.086e-6; 
    Mpv = ins.eth.Mpv;
    Mpp <<           0,            0,    -vN * f_RMh2, 
            vE * tl * f_clRNh,     0,  -vE*secl * f_RNh2, 
                     0,            0,           0;
    Ft.setZero();
    Ft.block<3,3>(0,0) = Maa; Ft.block<3,3>(0,3) = Mav; Ft.block<3,3>(0,6) = Map; Ft.block<3,3>(0,9) = -ins.Cnb;
    Ft.block<3,3>(3,0) = Mva; Ft.block<3,3>(3,3) = Mvv; Ft.block<3,3>(3,6) = Mvp; Ft.block<3,3>(3,12) = ins.Cnb;
                              Ft.block<3,3>(6,3) = Mpv; Ft.block<3,3>(6,6) = Mpp;
    Ft(19,20) = 1.0;  
    return true;
}                                                                                 

bool TCINS::seqEKF::timeUpdate()
{
    JacobianFt();
    Ft = Eigen::MatrixXd::Identity(21, 21) + Ft * ins.ts;
    P = Ft * P * Ft.transpose(); 
    P = P + Q * ins.ts; 
    isFirstMeasured = false;
    return true;
}

bool TCINS::seqEKF::measurementUpdate(double pseRange, double pseRangeRate)
{
    return true;
}

bool TCINS::seqEKF::blh2xyz(Vector3d& Pn)
{
    double sB = std::sin(Pn[0]); double cB = std::cos(Pn[0]);
    double sL = std::sin(Pn[1]); double cL = std::cos(Pn[1]);
    double H = Pn[2];
    double N = navGlv::Re / std::sqrt(1 - (navGlv::e2 * sB * sB));
    double NH = N + H;
    double X = NH * cB * cL;
    double Y = NH * cB * sL;
    double Z = (N * (1 - navGlv::e2) + H) * sB;
    r_ea_e << X, Y, Z;
    Cen <<   -sL,   -sB * cL,  cB * cL,
              cL,   -sB * sL,  cB * sL,
               0,       cB,      sB; 
    parXYZBLH <<      -NH * sB * cL,      -NH * cB * sL,  cB * cL,
                      -NH * sB * sL,       NH * cB * cL,  cB * sL,
                 (NH - navGlv::e2 * N) * cB ,     0 ,       sB;
    return true;
}

bool TCINS::seqEKF::blh2xyzWithoutPar(Vector3d& Pn)
{
    double sB = std::sin(Pn[0]); double cB = std::cos(Pn[0]);
    double sL = std::sin(Pn[1]); double cL = std::cos(Pn[1]);
    double H = Pn[2];
    double N = navGlv::Re / std::sqrt(1 - (navGlv::e2 * sB * sB));
    double NH = N + H;
    double X = NH * cB * cL;
    double Y = NH * cB * sL;
    double Z = (N * (1 - navGlv::e2) + H) * sB;
    r_ea_e << X, Y, Z;
    Cen <<   -sL,   -sB * cL,  cB * cL,
              cL,   -sB * sL,  cB * sL,
               0,       cB,      sB; 
    return true;
}

bool TCINS::seqEKF::JacobianHtOnlyRange(Vector3d &starPos)
{
    Ht.block<1,3>(0,6) = u_as_e.transpose() * parXYZBLH; // pos
    Ht.block<1,3>(0,15) = u_as_e.transpose() * ins.MpvCnb; // lever
    Ht(0,18) = u_as_e.transpose() * ins.Mpvvn; // delta T
    Ht(0,19) = 1; // ccorr
    return true;
}

bool TCINS::seqEKF::JacobianHt(Vector3d &starPos)
{
    Ht2.block<1,3>(0,6) = u_as_e.transpose() * parXYZBLH; // pos
    Ht2.block<1,3>(0,15) = u_as_e.transpose() * ins.MpvCnb; // lever
    Ht2(0,18) = u_as_e.transpose() * ins.Mpvvn; // delta T
    Ht2(0,19) = 1; // ccorr
    return true;
}

bool TCINS::seqEKF::seqEKFTCNHC() 
{
    tmpNHC_M = ins.Cnb.transpose();
    HtNHC.block<1,3>(0,3) = tmpNHC_M.block<1,3>(0,0);
    HtNHC.block<1,3>(1,3) = tmpNHC_M.block<1,3>(2,0);
    tmpNHC_M = -tmpNHC_M * vector2skew(ins.vn);
    HtNHC.block<1,3>(0,0) = tmpNHC_M.block<1,3>(0,0);
    HtNHC.block<1,3>(1,0) = tmpNHC_M.block<1,3>(2,0);
    
    // measurement update
    innov_covNHC = HtNHC * P * HtNHC.transpose() + NHCR;
    cross_corr_covNHC = P * HtNHC.transpose();
    KGainNHC = cross_corr_covNHC * innov_covNHC.inverse();
    tmpNHC_V = ins.Cnb.transpose()* ins.vn;
    Eigen::Vector2d tmpObs(tmpNHC_V(0),tmpNHC_V(2)); 
    X = KGainNHC * tmpObs;
    P = P - KGainNHC * innov_covNHC * KGainNHC.transpose();
    // feedback
    double angle = fRotAxis.norm();
    fRotAxis.normalize();
    Eigen::AngleAxisd phi(angle, fRotAxis);
    ins.qnb = Eigen::Quaterniond(phi) * ins.qnb;
    ins.vn = ins.vn - fVn;
    ins.pos = ins.pos - fPos;
    ins.biasGyro = ins.biasGyro + fBiasGyro;
    ins.biasAcc = ins.biasAcc + fBiasAcc;
    ins.lever = ins.lever + fLever;
    ins.dt = ins.dt + fDt;
    ins.ccorr = ins.ccorr + fCcorr;
    ins.cdrift = ins.cdrift + fCdrift;
    X.setZero();
    // sync atti
    ins.Cnb = ins.qnb.toRotationMatrix(); 
    ins.atti << std::asin(ins.Cnb(2,1)), 
                std::atan2(-ins.Cnb(2,0),ins.Cnb(2,2)),
                std::atan2(-ins.Cnb(0,1),ins.Cnb(1,1));
    if (ins.Cnb(2,1) > 0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    } else if(ins.Cnb(2,1) < -0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = -std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    }
    return true;
}

bool TCINS::seqEKF::seqMeasurementUpdate(double pseRange, Vector3d &starPos)
{
    if (!isFirstMeasured) {
        // compensate lever error
        ins.Cw = ins.Cnb * vector2skew(ins.web); 
        ins.MpvCnb = ins.eth.Mpv * ins.Cnb;
        ins.estPos = ins.pos + ins.MpvCnb * ins.lever;
        // ins.estPos = ins.pos;
        // compensate clock sync error
        ins.estPos = ins.estPos - ins.Mpvvn * ins.dt;
        blh2xyz(ins.estPos);
        isFirstMeasured = true;
    } else {
        // compensate lever error
        ins.Cw = ins.Cnb * vector2skew(ins.web); 
        ins.MpvCnb = ins.eth.Mpv * ins.Cnb;
        ins.estPos = ins.pos + ins.MpvCnb * ins.lever;
        // ins.estPos = ins.pos;
        // compensate clock sync error
        ins.estPos = ins.estPos - ins.Mpvvn * ins.dt;
        blh2xyzWithoutPar(ins.estPos);
    }
    delta_r = starPos - r_ea_e;
    double approx_range = delta_r.norm();
    u_as_e = delta_r / approx_range;
    ins.Cei = Eigen::Matrix3d::Identity() - (Omega_ie * approx_range / navGlv::lightspeed);
    delta_r = ins.Cei * starPos - r_ea_e;
    double obs_meas = delta_r.norm() + ins.ccorr;
    JacobianHtOnlyRange(starPos); 
    // measurement update
    double innov_cov = Ht * P * Ht.transpose() + R;
    cross_corr_cov = P * Ht.transpose();
    KGain = cross_corr_cov / innov_cov;
    X = KGain * (pseRange - obs_meas);
    P = P - KGain * innov_cov * KGain.transpose();
    // feedback
    double angle = fRotAxis.norm();
    fRotAxis.normalize();
    Eigen::AngleAxisd phi(angle, fRotAxis);
    ins.qnb = Eigen::Quaterniond(phi) * ins.qnb;
    ins.vn = ins.vn - fVn;
    ins.pos = ins.pos - fPos;
    ins.biasGyro = ins.biasGyro + fBiasGyro;
    ins.biasAcc = ins.biasAcc + fBiasAcc;
    ins.lever = ins.lever + fLever;
    ins.dt = ins.dt + fDt;
    ins.ccorr = ins.ccorr + fCcorr;
    ins.cdrift = ins.cdrift + fCdrift;
    X.setZero();
    // sync atti
    ins.Cnb = ins.qnb.toRotationMatrix(); 
    ins.atti << std::asin(ins.Cnb(2,1)), 
                std::atan2(-ins.Cnb(2,0),ins.Cnb(2,2)),
                std::atan2(-ins.Cnb(0,1),ins.Cnb(1,1));
    if (ins.Cnb(2,1) > 0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    } else if(ins.Cnb(2,1) < -0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = -std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    }
    return true;
}

TCINS::seqEKF::seqEKF(TCINS& tcins): X(Eigen::Matrix<double, 21, 1>::Zero()),ins(tcins), fDt(X(18)), fCcorr(X(19)), 
                     fCdrift(X(20)), Mpv(tcins.eth.Mpv), fRotAxis(X.segment<3>(0)), fVn(X.segment<3>(3)),
                     fPos(X.segment<3>(6)), fBiasGyro(X.segment<3>(9)), fBiasAcc(X.segment<3>(12)), fLever(X.segment<3>(15))
{
    Ft.setZero(); 
    Ht.setZero(); 
    Mp1.setZero(); 
    Mp2.setZero(); 
    Maa.setZero(); 
    Mav.setZero(); 
    Map.setZero(); 
    Mva.setZero(); 
    Mvv.setZero(); 
    Mvp.setZero(); 
    Mpp.setZero();
    Cen.setZero();
    HtNHC.setZero();
    parXYZBLH.setZero();
    P.setZero(); 
    Q.setZero(); 
    R = 0; 
    R2.setZero();
    r_ea_e.setZero();
    v_ea_e.setZero();
    u_as_e.setZero();
    Omega_ie.setZero();
    Omega_ie(0,1) = -navGlv::wie;
    Omega_ie(1,0) = navGlv::wie;
    isFirstMeasured = false;
    init();
}
    
TCINS::seqEKF::~seqEKF()
{
}

bool TCINS::seqBIVBEKF::init()
{
    double varOfGyroV = ins.deviationOfGyroV * ins.deviationOfGyroV;
    double varOfAccV = ins.deviationOfAccV * ins.deviationOfAccV;
    double varclock_freq_PSD = ins.clock_freq_PSD * ins.clock_freq_PSD;
    double varclock_phase_PSD = ins.clock_phase_PSD * ins.clock_phase_PSD;
    Q(0,0) = varOfGyroV; Q(1,1) = varOfGyroV; Q(2,2) = varOfGyroV; 
    Q(3,3) = varOfAccV; Q(4,4) = varOfAccV; Q(5,5) = varOfAccV;
    Q(19,19) = varclock_phase_PSD; Q(20,20)= varclock_freq_PSD;
    P(0,0) = ((10.0/3.0)*navGlv::deg2rad)*((10.0/3.0)*navGlv::deg2rad);
    P(1,1) = P(0,0); P(2,2) = P(0,0);
    P(3,3) = (0.15/3)*(0.15/3);
    P(4,4) = P(3,3); P(5,5) = P(3,3);
    P(6,6) = 1e-6*1e-6; P(7,7) = P(6,6); P(8,8) = (10.0/3.0) * (10.0/3.0);
    P(9,9) = ins.initialBiasG * navGlv::dph * ins.initialBiasG * navGlv::dph;
    P(10,10) = P(9,9);
    P(11,11) = P(9,9);
    P(12,12) = ins.initialBiasA * ins.initialBiasA; 
    P(13,13) = P(12,12); 
    P(14,14) = P(12,12); 
    P(15,15) = ins.lever[0] * ins.lever[0]; 
    P(16,16) = ins.lever[1] * ins.lever[1]; 
    P(17,17) = ins.lever[2] * ins.lever[2]; 
    P(18,18) = ins.dt * ins.dt;
    P(19,19) = ins.rx_clock_offset * ins.rx_clock_offset;
    P(20,20) = ins.rx_clock_drift * ins.rx_clock_drift;
    R = ins.pseudo_range_SD * ins.pseudo_range_SD;
    R2(0,0) = ins.pseudo_range_SD * ins.pseudo_range_SD;
    R2(1,1) = ins.range_rate_SD * ins.range_rate_SD;
    NHCR.setZero();
    NHCR(0,0) = ins.rNHC * ins.rNHC;
    NHCR(1,1) = NHCR(0,0);
    return true;
}

bool TCINS::seqBIVBEKF::JacobianFt()
{
    double tl = ins.eth.tl, secl = 1.0 / ins.eth.cl;
    double secl2 = secl * secl, wN = ins.eth.wnie[1], 
            wU = ins.eth.wnie[2], vE = ins.vn[0], vN = ins.vn[1];
	double f_RMh = ins.eth.f_RMh, f_RNh = ins.eth.f_RNh, f_clRNh = ins.eth.f_clRNh; 
	double f_RMh2 = f_RMh * f_RMh, f_RNh2 = f_RNh * f_RNh;
    Matrix3d Avn = vector2skew(ins.vn);
    Mp1 <<  0,      0,       0,
           -wU,     0,       0,
            wN,     0,       0;
    Mp2 <<            0,          0,      vN * f_RMh2,
                      0,          0,     -vE * f_RNh2,
           vE * secl2 *  f_RNh,   0,   -vE * tl * f_RNh2;
    Vector3d tmpVec = -ins.eth.wnin;
    Maa = vector2skew(tmpVec);
    Mav <<       0,       -f_RMh,    0, 
               f_RNh,        0,      0, 
            tl * f_RNh,      0,      0;
    Map = Mp1 + Mp2;
    Mva = vector2skew(ins.fn);
    tmpVec = ins.eth.wnie + ins.eth.wnin;
    Mvv = Avn * Mav - vector2skew(tmpVec);
    Mvp = Avn * (Mp1 + Map);
    double scl = ins.eth.sl*ins.eth.cl;
    Mvp(2,0) = Mvp(2,0) - navGlv::g * (5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * ins.eth.sl2 * scl);
    Mvp(2,2) = Mvp(2,2) + 3.086e-6; 
    Mpv = ins.eth.Mpv;
    Mpp <<           0,            0,    -vN * f_RMh2, 
            vE * tl * f_clRNh,     0,  -vE*secl * f_RNh2, 
                     0,            0,           0;
    Ft.setZero();
    Ft.block<3,3>(0,0) = Maa; Ft.block<3,3>(0,3) = Mav; Ft.block<3,3>(0,6) = Map; Ft.block<3,3>(0,9) = -ins.Cnb;
    Ft.block<3,3>(3,0) = Mva; Ft.block<3,3>(3,3) = Mvv; Ft.block<3,3>(3,6) = Mvp; Ft.block<3,3>(3,12) = ins.Cnb;
                              Ft.block<3,3>(6,3) = Mpv; Ft.block<3,3>(6,6) = Mpp;
    Ft(19,20) = 1.0;  
    return true;
}                                                                                 

bool TCINS::seqBIVBEKF::timeUpdate()
{
    JacobianFt();
    Ft = Eigen::MatrixXd::Identity(21, 21) + Ft * ins.ts;
    P = Ft * P * Ft.transpose(); 
    P = P + Q * ins.ts; 
    isFirstMeasured = false;
    return true;
}

bool TCINS::seqBIVBEKF::blh2xyzWithoutPar(Vector3d& Pn)
{
    double sB = std::sin(Pn[0]); double cB = std::cos(Pn[0]);
    double sL = std::sin(Pn[1]); double cL = std::cos(Pn[1]);
    double H = Pn[2];
    double N = navGlv::Re / std::sqrt(1 - (navGlv::e2 * sB * sB));
    double NH = N + H;
    double X = NH * cB * cL;
    double Y = NH * cB * sL;
    double Z = (N * (1 - navGlv::e2) + H) * sB;
    r_ea_e << X, Y, Z;
    Cen <<   -sL,   -sB * cL,  cB * cL,
              cL,   -sB * sL,  cB * sL,
               0,       cB,      sB; 
    return true;
}

bool TCINS::seqBIVBEKF::blh2xyz(Vector3d& Pn)
{
    double sB = std::sin(Pn[0]); double cB = std::cos(Pn[0]);
    double sL = std::sin(Pn[1]); double cL = std::cos(Pn[1]);
    double H = Pn[2];
    double N = navGlv::Re / std::sqrt(1 - (navGlv::e2 * sB * sB));
    double NH = N + H;
    double X = NH * cB * cL;
    double Y = NH * cB * sL;
    double Z = (N * (1 - navGlv::e2) + H) * sB;
    r_ea_e << X, Y, Z;
    Cen <<   -sL,   -sB * cL,  cB * cL,
              cL,   -sB * sL,  cB * sL,
               0,       cB,      sB; 
    parXYZBLH <<      -NH * sB * cL,      -NH * cB * sL,  cB * cL,
                      -NH * sB * sL,       NH * cB * cL,  cB * sL,
                 (NH - navGlv::e2 * N) * cB ,     0 ,       sB;
    return true;
}

bool TCINS::seqBIVBEKF::JacobianHtOnlyRange(Vector3d &starPos)
{
    Ht.block<1,3>(0,6) = u_as_e.transpose() * parXYZBLH; // pos
    Ht.block<1,3>(0,15) = u_as_e.transpose() * ins.MpvCnb; // lever
    Ht(0,18) = u_as_e.transpose() * ins.Mpvvn; // delta T
    Ht(0,19) = 1; // ccorr
    return true;
}

bool TCINS::seqBIVBEKF::JacobianHt(Vector3d &starPos)
{
    Ht2.block<1,3>(0,6) = u_as_e.transpose() * parXYZBLH; // pos
    Ht2.block<1,3>(0,15) = u_as_e.transpose() * ins.MpvCnb; // lever
    Ht2(0,18) = u_as_e.transpose() * ins.Mpvvn; // delta T
    Ht2(0,19) = 1; // ccorr
    return true;
}

bool TCINS::seqBIVBEKF::seqBIVBUpdateLUT()
{
    for (auto vmInfo: BIVBMLUT) {
        if (vmInfo.second.isValid == VBMINFOSTATE::curUsed) {
            vmInfo.second.isValid = VBMINFOSTATE::noDefined;
        } else {
             vmInfo.second.isValid = VBMINFOSTATE::init;
        }
    }
}

bool TCINS::seqBIVBEKF::seqBIVBEKFTCNHC() 
{
    tmpNHC = ins.Cnb.transpose();
    HtNHC.block<1,3>(0,3) = tmpNHC.block<1,3>(0,0);
    HtNHC.block<1,3>(1,3) = tmpNHC.block<1,3>(2,0);
    tmpNHC = -tmpNHC * vector2skew(ins.vn);
    HtNHC.block<1,3>(0,0) = tmpNHC.block<1,3>(0,0);
    HtNHC.block<1,3>(1,0) = tmpNHC.block<1,3>(2,0);
    // measurement update
    innov_covNHC = HtNHC * P * HtNHC.transpose() + NHCR;
    cross_corr_covNHC = P * HtNHC.transpose();
    KGainNHC = cross_corr_covNHC * innov_covNHC.inverse();
    tmpNHC_V = ins.Cnb.transpose()* ins.vn;
    Eigen::Vector2d tmpObs(tmpNHC_V(0),tmpNHC_V(2)); 
    X = KGainNHC * tmpObs;
    P = P - KGainNHC * innov_covNHC * KGainNHC.transpose();
    // feedback
    double angle = fRotAxis.norm();
    fRotAxis.normalize();
    Eigen::AngleAxisd phi(angle, fRotAxis);
    ins.qnb = Eigen::Quaterniond(phi) * ins.qnb;
    ins.vn = ins.vn - fVn;
    ins.pos = ins.pos - fPos;
    ins.biasGyro = ins.biasGyro + fBiasGyro;
    ins.biasAcc = ins.biasAcc + fBiasAcc;
    ins.lever = ins.lever + fLever;
    ins.dt = ins.dt + fDt;
    ins.ccorr = ins.ccorr + fCcorr;
    ins.cdrift = ins.cdrift + fCdrift;
    X.setZero();
    // sync atti
    ins.Cnb = ins.qnb.toRotationMatrix(); 
    ins.atti << std::asin(ins.Cnb(2,1)), 
                std::atan2(-ins.Cnb(2,0),ins.Cnb(2,2)),
                std::atan2(-ins.Cnb(0,1),ins.Cnb(1,1));
    if (ins.Cnb(2,1) > 0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    } else if(ins.Cnb(2,1) < -0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = -std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    }
    return true;
}

bool TCINS::seqBIVBEKF::seqBIVBMeasurementUpdate(uint16_t prn, double pseRange, Vector3d &starPos)
{
    if (!isFirstMeasured) {
        // compensate lever error -fix
        ins.Cw = ins.Cnb * vector2skew(ins.web); 
        ins.MpvCnb = ins.eth.Mpv * ins.Cnb;
        ins.estPos = ins.pos + ins.MpvCnb * ins.lever;
        ins.estPos = ins.pos;
        // compensate clock sync error - to fix estPos
        ins.estPos = ins.estPos - ins.Mpvvn * ins.dt;
        blh2xyz(ins.estPos);
        isFirstMeasured = true;
    } else {
        // compensate lever error
        ins.Cw = ins.Cnb * vector2skew(ins.web); 
        ins.MpvCnb = ins.eth.Mpv * ins.Cnb;
        ins.estPos = ins.pos + ins.MpvCnb * ins.lever;
        // ins.estPos = ins.pos;
        // compensate clock sync error
        ins.estPos = ins.estPos - ins.Mpvvn * ins.dt;
        blh2xyzWithoutPar(ins.estPos);
    }
    // todo: 
    // search corresponding VB measurement information & update VBMLUT
    if (BIVBMLUT.find(prn) != BIVBMLUT.end()) {
        // find VB information
        if (BIVBMLUT[prn].isValid == VBMINFOSTATE::noDefined) {
            BIVBMLUT[prn].isValid = VBMINFOSTATE::curUsed;
            BIVBMLUT[prn].et = VBe0;
            BIVBMLUT[prn].ft = VBf0;
            BIVBMLUT[prn].Zt = VBzt0;
        } else {
            BIVBMLUT[prn].isValid = VBMINFOSTATE::curUsed;
            BIVBMLUT[prn].IGa = VBIGa0;
            BIVBMLUT[prn].IGb = R;
            BIVBMLUT[prn].et = VBe0;
            BIVBMLUT[prn].ft = VBf0;
            BIVBMLUT[prn].Zt = VBzt0;
        }
        
    } else {
        BIVBMLUT[prn].isValid = VBMINFOSTATE::curUsed;
        BIVBMLUT[prn].IGa = VBIGa0;
        BIVBMLUT[prn].IGb = R;
        BIVBMLUT[prn].et = VBe0;
        BIVBMLUT[prn].ft = VBf0;
        BIVBMLUT[prn].Zt = VBzt0;
    }
    TCBIVBMINFO &curVBMInfo = BIVBMLUT[prn];
    curVBMInfo.IGa = 0.5 + curVBMInfo.IGa;
    double tmpIGb = curVBMInfo.IGb;
    Matrix<double, 21, 21> tmpP = P;
    delta_r = starPos - r_ea_e;
    double approx_range = delta_r.norm();
    u_as_e = delta_r / approx_range;
    // X is always zero vector
    ins.Cei = Eigen::Matrix3d::Identity() - (Omega_ie * approx_range / navGlv::lightspeed); // approx sim
    delta_r = ins.Cei * starPos - r_ea_e;
    double obs_meas = delta_r.norm() + ins.ccorr; // yt(X+)
    JacobianHtOnlyRange(starPos); 
    // VB iteration
    for (size_t i = 0; i < VBmaxit_d; i++) {
        if (curVBMInfo.Zt > VBZtTH) {
            // od is invalid
            Matrix<double, 21, 1> Xpre = X;
            curVBMInfo.R = curVBMInfo.IGb / curVBMInfo.IGa;            
            double innov_cov = Ht * tmpP * Ht.transpose() + curVBMInfo.R / curVBMInfo.Zt;
            KGain = tmpP * Ht.transpose() / innov_cov;
            X = KGain * (pseRange - obs_meas);
            P = tmpP - KGain * innov_cov * KGain.transpose();
            // estimate yt(Xi)
            dr_ea_e = Cen * (fPos + ins.Mpvvn * fDt - ins.MpvCnb * fLever);
            // dr_ea_e = Cen * (  fPos );  // will reduce time consumption about 0.01s
            delta_r = starPos - r_ea_e - dr_ea_e;
            approx_range = delta_r.norm();
            ins.Cei = Eigen::Matrix3d::Identity() - (Omega_ie * approx_range / navGlv::lightspeed);
            delta_r = ins.Cei * starPos - r_ea_e - dr_ea_e;
            double ytXi = delta_r.norm() + ins.ccorr + fCcorr;
            double tmpinnov2 = (pseRange - ytXi) * (pseRange - ytXi);
            double tmpHPH = Ht * P * Ht.transpose();
            double VBBt = tmpinnov2 +tmpHPH;
            double tmpp1 = exp(gsl_sf_psi(curVBMInfo.et) - gsl_sf_psi(curVBMInfo.et + curVBMInfo.ft) - (0.5 * VBBt / curVBMInfo.R));
            double tmpp0 = exp(gsl_sf_psi(curVBMInfo.ft) - gsl_sf_psi(curVBMInfo.et + curVBMInfo.ft));
            curVBMInfo.Zt = tmpp1 / (tmpp1 + tmpp0);
            curVBMInfo.IGb = tmpIGb +  0.5 * curVBMInfo.Zt * (VBBt);
            curVBMInfo.et = VBe0 + curVBMInfo.Zt;
            curVBMInfo.ft = VBf0 + 1 - curVBMInfo.Zt;
            Matrix<double, 21, 1> err = X - Xpre;
            if (err.norm()  < VBerr_TH) {
                break;
            }
            
        } else {
            // od is valid
            X.setZero();
            P = tmpP;
            curVBMInfo.IGb = tmpIGb;
            curVBMInfo.IGa = curVBMInfo.IGa - 0.5;
            curVBMInfo.R = curVBMInfo.IGb / curVBMInfo.IGa;
            break;
        }
        
    }
    // feedback
    double angle = fRotAxis.norm();
    fRotAxis.normalize();
    Eigen::AngleAxisd phi(angle, fRotAxis);
    ins.qnb = Eigen::Quaterniond(phi) * ins.qnb;
    ins.vn = ins.vn - fVn;
    ins.pos = ins.pos - fPos;
    ins.biasGyro = ins.biasGyro + fBiasGyro;
    ins.biasAcc = ins.biasAcc + fBiasAcc;
    ins.lever = ins.lever + fLever;
    ins.dt = ins.dt + fDt;
    ins.ccorr = ins.ccorr + fCcorr;
    ins.cdrift = ins.cdrift + fCdrift;
    X.setZero();
    // sync atti
    ins.Cnb = ins.qnb.toRotationMatrix(); 
    ins.atti << std::asin(ins.Cnb(2,1)), 
                std::atan2(-ins.Cnb(2,0),ins.Cnb(2,2)),
                std::atan2(-ins.Cnb(0,1),ins.Cnb(1,1));
    if (ins.Cnb(2,1) > 0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    } else if(ins.Cnb(2,1) < -0.999999) {
        ins.atti(1) = 0;
        ins.atti(1) = -std::atan2(ins.Cnb(0,2),ins.Cnb(0,0));
    }
    return true;
}

TCINS::seqBIVBEKF::seqBIVBEKF(TCINS& tcins): X(Eigen::Matrix<double, 21, 1>::Zero()),ins(tcins), fDt(X(18)), fCcorr(X(19)), 
                     fCdrift(X(20)), Mpv(tcins.eth.Mpv), fRotAxis(X.segment<3>(0)), fVn(X.segment<3>(3)),
                     fPos(X.segment<3>(6)), fBiasGyro(X.segment<3>(9)), fBiasAcc(X.segment<3>(12)), fLever(X.segment<3>(15))
{
    Ft.setZero(); 
    Ht.setZero(); 
    Mp1.setZero(); 
    Mp2.setZero(); 
    Maa.setZero(); 
    Mav.setZero(); 
    Map.setZero(); 
    Mva.setZero(); 
    Mvv.setZero(); 
    Mvp.setZero(); 
    Mpp.setZero();
    Cen.setZero();
    parXYZBLH.setZero();
    P.setZero(); 
    Q.setZero(); 
    R = 0; 
    R2.setZero();
    r_ea_e.setZero();
    v_ea_e.setZero();
    u_as_e.setZero();
    Omega_ie.setZero();
    Omega_ie(0,1) = -navGlv::wie;
    Omega_ie(1,0) = navGlv::wie;
    isFirstMeasured = false;
    VBpho_d = 1 - exp(-4.0);
    init();
}
    
TCINS::seqBIVBEKF::~seqBIVBEKF()
{
}

const std::unordered_map<uint16_t, TCBIVBMINFO>& TCINS::seqBIVBEKF::getVBMInfo() const 
{
    return BIVBMLUT;
}
