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

#ifndef _TCINS_
#define _TCINS_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ins.h"
#include "BASEElement.h"
#include <string>
#include <cmath>
#include <unordered_map>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::Matrix;
// using Eigen::Map;

enum class VBMINFOSTATE {
	init,
	curUsed,
	noDefined
};

typedef struct tagTCINSSATE
{
    const Vector3d& atti; // 3 yaw pitch roll
    const Vector3d& vn; // 6
    const Vector3d& pos; // 9
    const Vector3d& biasG; // 12
    const Vector3d& biasA; // 15
    const Vector3d& lever; // 18
    const double& dt; // 19
    const double& ccorr; // 20 
    const double& cdrift; //21
}TCINSSTATE;

typedef struct tagTCBIVBMInfo {
    VBMINFOSTATE isValid;
    double IGb;
    double IGa;
    double et;
    double ft;
    double R;
    double Zt;
}TCBIVBMINFO;

class TCINS : public INS
{
private:
    /* data */
    Matrix<double, 3, 3> Cei; 
    Vector3d tmpVn, tmpPos;
    double ccorr, cdrift;
    double rx_clock_offset, rx_clock_drift;
    double range_rate_SD, pseudo_range_SD, clock_freq_PSD, clock_phase_PSD;
public:
    TCINS(/* args */);
    ~TCINS();
    bool imuCompensate(Vector3d &deltaGyro, Vector3d &deltaAcc); 
    bool insConfig(const std::string& configFilePath);
    bool insUpdate(Vector3d &gyro, Vector3d &acc);
    bool insOutput();
    bool insLogFile();
    bool xyz2blh(Vector3d& xyz, Vector3d& blh);
    bool setPos(Vector3d &gnssPos);
    bool getECEFFromSats(double& psr, Vector3d& deltaGyro, Vector3d&xECEF);
    TCINSSTATE getInsState() const;
    bool isFirstIMUData, isFileOpen;
    class seqEKF
    {
    public:
        Matrix<double, 21, 1> X, KGain, cross_corr_cov;
        Matrix<double, 21, 21> P, Q; 
        Matrix<double, 21, 2> KGainNHC, cross_corr_covNHC;
        Matrix<double, 2, 2> innov_covNHC;
        Matrix<double, 3, 3> tmpNHC_M; 
        Matrix<double, 2, 2> NHCR;
        Vector3d tmpNHC_V;
        double R; 
        Matrix<double, 2, 2> R2;
        Matrix<double, 21, 21>  Ft; 
        Matrix<double, 1, 21>  Ht; 
        Matrix<double, 2, 21>  Ht2; 
        Matrix<double, 2, 21>  HtNHC;
        decltype(X.segment<3>(0)) fRotAxis, fVn, fPos, fBiasGyro, fBiasAcc, fLever;
        // Matrix<double, 3, 1> fRotAxis, fVn, fPos, fBiasGyro, fBiasAcc, fLever;
        double &fDt, &fCcorr, &fCdrift;
        // Eigen::Ref<Eigen::Matrix<double, 3, 3>> Mpv;
        Matrix3d &Mpv;
        TCINS &ins;
        Matrix3d Mp1, Mp2, Maa, Mav, Map, Mva, Mvv, Mvp, Mpp, Cen, Omega_ie, parXYZBLH;
        Vector3d r_ea_e, v_ea_e, u_as_e, delta_r;
        bool isFirstMeasured;
        bool blh2xyz(Vector3d& Pn);
        bool blh2xyzWithoutPar(Vector3d& Pn);
        bool JacobianFt();
        bool JacobianHtOnlyRange(Vector3d &starPos); 
        bool JacobianHt(Vector3d &starPos); 
    public:
        seqEKF(TCINS& tcins);//
        ~seqEKF();
        bool init();
        bool timeUpdate();
        bool seqMeasurementUpdate(double pseRange, Vector3d &starPos);
        bool seqEKFTCNHC();
        bool measurementUpdate(double pseRange, double pseRangeRate);
    };
    class seqBIVBEKF
    {
    public:
        
        Matrix<double, 21, 1> X, KGain, cross_corr_cov;
        Matrix<double, 21, 21> P, Q; 
        Matrix<double, 21, 2> KGainNHC, cross_corr_covNHC;
        Matrix<double, 2, 2> innov_covNHC;
        Matrix<double, 3, 3> tmpNHC; 
        Matrix<double, 2, 2> NHCR;
        Vector3d tmpNHC_V;
        double R; 
        double VBpho_d;
        double VBerr_TH{1e-5};
        const double VBZtTH = 0.1;
        const double VBe0 = 0.9;
        const double VBf0 = 0.1;
        const double VBzt0 = 1;
        const double VBIGa0 = 1;
        // const double VBIGb0 = 1;
        uint32_t VBmaxit_d{100};
        std::unordered_map<uint16_t, TCBIVBMINFO> BIVBMLUT;
        Matrix<double, 2, 2> R2;
        Matrix<double, 21, 21>  Ft; 
        Matrix<double, 1, 21>  Ht; 
        Matrix<double, 2, 21>  Ht2; 
        Matrix<double, 2, 21>  HtNHC;
        decltype(X.segment<3>(0)) fRotAxis, fVn, fPos, fBiasGyro, fBiasAcc, fLever;
        double &fDt, &fCcorr, &fCdrift;
        Matrix3d &Mpv;
        TCINS &ins;
        Matrix3d Mp1, Mp2, Maa, Mav, Map, Mva, Mvv, Mvp, Mpp, Cen, Omega_ie, parXYZBLH;
        Vector3d r_ea_e, v_ea_e, u_as_e, delta_r, dr_ea_e;
        bool isFirstMeasured;
        bool blh2xyz(Vector3d& Pn);
        bool blh2xyzWithoutPar(Vector3d& Pn);
        bool JacobianFt();
        bool JacobianHtOnlyRange(Vector3d &starPos); 
        bool JacobianHt(Vector3d &starPos); 
    public:
        seqBIVBEKF(TCINS& tcins);//
        ~seqBIVBEKF();
        bool init();
        bool timeUpdate();
        const std::unordered_map<uint16_t, TCBIVBMINFO>& getVBMInfo() const;
        bool seqBIVBMeasurementUpdate(uint16_t prn, double pseRange, Vector3d &starPos);
        bool seqBIVBEKFTCNHC();
        bool seqBIVBUpdateLUT();
    };
};


class TCINSCreator : public INSCreator
{
private:
    /* data */
public:
    INS* CreateINS(/* args */) override {return new TCINS;}
};

#endif
