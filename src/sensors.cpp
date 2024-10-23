#include "sensors.hpp"
#include "Manager.hpp"
#include "sensorsMsg.hpp"
#include "BASEElement.h"
#include "main.hpp"
#include <thread>
#include <unordered_map>
#include <ctime>
#include <iomanip>

#include <Eigen/Dense>

using Eigen::Vector3d;

#define SENSORS_IMU_SAMPLE_FREQ GLV_IMU_SAMPLES

SENSORS::SENSORS(const std::string& imuPath, const std::string& gnssPath):imuSTIM300(imuPath), gnssUM982(gnssPath, 921600)
{

}

SENSORS::~SENSORS()
{

}

bool SENSORS::setIMUTopicName(const std::string& imuTopicName)
{
    if (imuTopicName.empty()) {
        return false;
    }
    // todo: delete old topic
    imuPub.setTopics(imuTopicName);
    return true;
}

bool SENSORS::setGNSSTopicName(const std::string& gnssTopicName)
{
    if (gnssTopicName.empty()) {
        return false;
    }
    // todo: delete old topic
    gnssPub.setTopics(gnssTopicName);
    return true;
}

bool SENSORS::setLogFile(const std::string& logFilePath)
{
    if (logFilePath.empty() ) {
        imuLogFile.close();
        gnssLogFile1.close();
        gnssLogFile2.close();
        isNeedSensorsLog = false;
        return true;
    }
    imuLogFile.close();
    gnssLogFile1.close();
    std::string imuFile = logFilePath + "imuRAW.txt";
    std::string gnssFile1 = logFilePath + "gnssRAW.txt";
    std::string gnssFile2 = logFilePath + "gnssBESTNAVRAW.txt";
    imuLogFile.open(imuFile);
    gnssLogFile1.open(gnssFile1);
    gnssLogFile2.open(gnssFile2);
    if (imuLogFile.is_open()&&gnssLogFile1.is_open()) {
        isNeedSensorsLog = true;
        imuLogFile << "lcTime Gyro_x Gyro_y Gyro_z Acc_x Acc_y Acc_z" << std::endl;
        gnssLogFile1 << "lcTime PRN type freqType PSR PSRSTD CN0 locktime satclkDly lonoDly tropDly lat lon alt" << std::endl;
        gnssLogFile2 << "lcTime locType hgt lat lon psrHgt psrLat psrLon undu psrSvs psrSolnsvs psrVN psrVE psrVG gdop pdop hdop htdop tdop cutoff PRNnums PRNlist" << std::endl;
    } else {
        return false;
    }
    return true;
}


bool SENSORS::setErrLogFile(const std::string& errLogPath)
{
    if (errLogPath.empty() ) {
        errLogFile.close();
        return true;
    }
    errLogFile.close();
    errLogFile.open(errLogPath);
    if (errLogFile.is_open()) {
        errLogFile << "Event Log File" << std::endl;
    } else {
        return false;
    }
    return true;
}

void SENSORS::listenIMU()
{
    // todo: check has topic
    IMUData data;
    while (true) {
        
        if (imuSTIM300.listen(data) > 0) {
            std::shared_ptr<SENSORMSG_IMUDATA> pxIMUData = std::make_shared<SENSORMSG_IMUDATA>();
            pxIMUData->gyro[0] = data.gx * navGlv::deg2rad / SENSORS_IMU_SAMPLE_FREQ;
            pxIMUData->gyro[1] = data.gy * navGlv::deg2rad / SENSORS_IMU_SAMPLE_FREQ;
            pxIMUData->gyro[2] = data.gz * navGlv::deg2rad / SENSORS_IMU_SAMPLE_FREQ;
            pxIMUData->acc[0] = data.ax * navGlv::g / SENSORS_IMU_SAMPLE_FREQ;
            pxIMUData->acc[1] = data.ay * navGlv::g / SENSORS_IMU_SAMPLE_FREQ;
            pxIMUData->acc[2] = data.az * navGlv::g / SENSORS_IMU_SAMPLE_FREQ;
            // imuPub.publish(pxIMUData);
            if (isNeedSensorsLog) {
                imuLogFile << std::scientific<< std::setprecision(14);
                imuLogFile << uiSync << ' '  << pxIMUData->gyro[0] << ' ' << pxIMUData->gyro[1] << ' ' << pxIMUData->gyro[2] << ' '
                                      << pxIMUData->acc[0] << ' ' << pxIMUData->acc[1] << ' ' << pxIMUData->acc[2] << std::endl;
            }
        } else {
            errLogFile << std::time(nullptr)  << " IMU parse error" << std::endl;
            // std::cout << std::time(nullptr)  << " IMU parse error" << std::endl;
        }
    }
}

bool SENSORS::parseSYSTYPEandPRN(const uint32_t & orignalPrn, uint32_t& prn, uint32_t& type)
{
    bool ret = false;
    if (orignalPrn <= 32) {
        type = (uint32_t)GNSSSYSTYPE::GPS;
        prn = orignalPrn;
        ret = true;
    } else if (orignalPrn <= 42) {
        type = (uint32_t)GNSSSYSTYPE::QZSS;
        prn = orignalPrn + 160;
        ret = true;
    } else if (orignalPrn <= 66) {
        type = (uint32_t)GNSSSYSTYPE::GLONASS;
        prn = orignalPrn - 5;
        ret = true;
    } else if (orignalPrn <= 74) {
        type = (uint32_t)GNSSSYSTYPE::NAVIC;
        prn = orignalPrn - 66;
        ret = true;
    } else if (orignalPrn <= 110) {
        type = (uint32_t)GNSSSYSTYPE::GALILEIO;
        prn = orignalPrn - 74;
        ret = true;
    } else if (orignalPrn <= 117) {
        type = (uint32_t)GNSSSYSTYPE::NAVIC;
        prn = orignalPrn - 110;
        ret = true;
    } else if (orignalPrn <= 158) {
        type = (uint32_t)GNSSSYSTYPE::SBAS;
        prn = orignalPrn;
        ret = true;
    } else if (orignalPrn <= 223) {
        type = (uint32_t)GNSSSYSTYPE::BDS;
        prn = orignalPrn - 160;
        ret = true;
    } else {
        type = (uint32_t)GNSSSYSTYPE::UNDEFINED;
        prn = orignalPrn;
        ret = false;
    }
    return ret;
}

void SENSORS::listenGNSSTC()
{
    // todo: check has topic
    void* pxRecGnssData = nullptr;
    EUNIOBSVMData *pxOBSVMData = nullptr;
    EUNISATECEFData *pxSATECEFData = nullptr;
    EUNIPVTSLNData *pxEUNIPVTSLNData = nullptr;
    EUNISATDOPData *pxEUNISATDOPData = nullptr;
    NMEAGGAData *pxNMEAGGAData = nullptr;
    uint32_t uiSyncBitFlag = 0;
    GNSSMSGTYPE msgType; 
    std::unordered_map<uint32_t, SENSORMSG_SATDATA> satECEFLUT;
    std::unordered_map<uint32_t, bool> trackSatLUT;
    while (true) {
        gnssUM982.listen(msgType, pxRecGnssData);
        switch (msgType)
        {
        case GNSSMSGTYPE::NMEA_XXGGA:
            pxNMEAGGAData = static_cast<NMEAGGAData*>(pxRecGnssData);
            break;
        case GNSSMSGTYPE::EUNI_OBSVM:
            if (uiSyncBitFlag == 0x03) {
                pxOBSVMData = static_cast<EUNIOBSVMData*>(pxRecGnssData);
                uiSyncBitFlag = 0;
            }
            break;
        case GNSSMSGTYPE::EUNI_STATECEF:
            pxSATECEFData = static_cast<EUNISATECEFData*>(pxRecGnssData); 
            uiSyncBitFlag |= 0x01;
            break;
        case GNSSMSGTYPE::EUNI_PVTSLN:
            pxEUNIPVTSLNData = static_cast<EUNIPVTSLNData*>(pxRecGnssData); 
            uiSyncBitFlag |= 0x02;
            break;
        case GNSSMSGTYPE::EUNI_STADOP:
            pxEUNISATDOPData = static_cast<EUNISATDOPData*>(pxRecGnssData); 
            break;
        default:
            break;
        }
        if (pxEUNISATDOPData != nullptr) {

            pxEUNISATDOPData = nullptr;
        }
        if (pxOBSVMData != nullptr && pxSATECEFData != nullptr && pxNMEAGGAData != nullptr && pxEUNIPVTSLNData != nullptr) {
            if (pxSATECEFData->satNum > 4) {
                if (isNeedSensorsLog) {
                    gnssLogFile2 << std::scientific<< std::setprecision(14);
                    // lcTime locType hgt lat lon psrHgt psrLat psrLon undu psrSvs psrSolnsvs psrVN psrVE psrVG gdop pdop hdop htdop tdop cutoff PRNnums PRNlist
                    gnssLogFile2 << uiSync << ' ' << pxEUNIPVTSLNData->bestpos_type << ' ' << pxEUNIPVTSLNData->bestpos_hgt << ' ' 
                        << pxEUNIPVTSLNData->bestpos_lat << ' ' << pxEUNIPVTSLNData->bestpos_lon << ' ' << pxEUNIPVTSLNData->psrpos_hgt << ' '
                        << pxEUNIPVTSLNData->psrpos_lat << ' ' << pxEUNIPVTSLNData->psrpos_lon << ' ' << pxEUNIPVTSLNData->undulation << ' '
                        << (int)pxEUNIPVTSLNData->psrpos_svs << ' ' << (int)pxEUNIPVTSLNData->psrpos_solnsvs << ' ' << pxEUNIPVTSLNData->psrvel_north << ' '
                        << pxEUNIPVTSLNData->psrvel_east << ' ' << pxEUNIPVTSLNData->psrvel_ground << ' ' << pxEUNIPVTSLNData->gdop << ' ' 
                        << pxEUNIPVTSLNData->pdop << ' ' << pxEUNIPVTSLNData->hdop << ' ' << pxEUNIPVTSLNData->htdop << ' '
                        << pxEUNIPVTSLNData->tdop << ' ' << pxEUNIPVTSLNData->cutoff << ' ' << pxEUNIPVTSLNData->PRNnums << ' ';
                }
                for (size_t i = 0; i < pxEUNIPVTSLNData->PRNnums; i++) {
                    if (isNeedSensorsLog) {
                        gnssLogFile2 << pxEUNIPVTSLNData->prns[i] << ' ';
                    }
                    uint32_t tmpprn = 0, tmpType = 0;
                    if (parseSYSTYPEandPRN(pxEUNIPVTSLNData->prns[i], tmpprn, tmpType)) {
                        trackSatLUT[tmpprn + (tmpType << 8)] = true;
                    } 
                }
                if (isNeedSensorsLog) {
                    gnssLogFile2 << std::endl;
                }

                // std::cout << "STATECEF_nums: " << pxSATECEFData->satNum << " OBSVM_nums: " <<pxOBSVMData->obsNum  << std::endl;
                // update Sat ECEF map
                for (size_t i = 0; i < pxSATECEFData->satNum; i++) {
                    uint32_t tmpKey = pxSATECEFData->satECEFInfo[i].prn + ((pxSATECEFData->satECEFInfo[i].sysType) << 8);
                    satECEFLUT[tmpKey].satECEF = 
                                    Vector3d(pxSATECEFData->satECEFInfo[i].satCoordX, 
                                             pxSATECEFData->satECEFInfo[i].satCoordY,
                                             pxSATECEFData->satECEFInfo[i].satCoordZ);
                    satECEFLUT[tmpKey].satClk = pxSATECEFData->satECEFInfo[i].satClk;
                    satECEFLUT[tmpKey].lonoDly = pxSATECEFData->satECEFInfo[i].lonoDly;
                    satECEFLUT[tmpKey].tropDly = pxSATECEFData->satECEFInfo[i].tropDly; 
                    satECEFLUT[tmpKey].isValid = true;
                }

                // search obsvm correspongding ECEF when info is valid
                for (size_t i = 0; i < pxOBSVMData->obsNum; i++) {
                    if ((pxOBSVMData->obsSVInfo[i].trStatus&EUNI_OBSVM_TRSTATUS_PSR_VALID_MASK) ==
                                    EUNI_OBSVM_TRSTATUS_PSR_VALID_MASK) {
                        uint32_t gnssType = (pxOBSVMData->obsSVInfo[i].trStatus & EUNI_OBSVM_TRSTATUS_SYS_MASK) >> 16;
                        uint32_t freqType = (pxOBSVMData->obsSVInfo[i].trStatus & EUNI_OBSVM_TRSTATUS_CHL_MASK) >> 21;
                        uint32_t tmpKey = pxOBSVMData->obsSVInfo[i].prn + (gnssType<<8);
                        if (satECEFLUT.find(tmpKey) == satECEFLUT.end()) {
                            // std::cout << "not found corresponding sats ECEF" << std::endl;
                            continue;
                        } 
                        if (satECEFLUT[tmpKey].isValid != true) {
                            // std::cout << "invaild sat ECEF for SYS: " << gnssType << " PRN: " <<  (int)(pxOBSVMData->obsSVInfo[i].prn) << std::endl;
                            continue;
                        }
                        if (trackSatLUT.find(tmpKey) == trackSatLUT.end()) {
                            // std::cout << "not found corresponding tracked sats" << std::endl;
                            continue;
                        } 
                        if (trackSatLUT[tmpKey] != true) {
                            // std::cout << "not tracked prn for SYS: " << gnssType << " PRN: " <<  (int)(pxOBSVMData->obsSVInfo[i].prn) << std::endl;
                            continue;
                        }
                        
                        double freq = 0.0;
                        bool ret = false;
                        ret = gnssUM982.getGNSSFreq((GNSSSYSTYPE)gnssType, freqType, freq);
                        if (ret) {
                            if (freq == 1602) {
                                freq = freq + pxOBSVMData->obsSVInfo[i].sysFreq * 0.5625;
                            } 
                            if (freq == 1246) {
                                freq = freq + pxOBSVMData->obsSVInfo[i].sysFreq * 0.4375;
                            }
                            freq = freq * 1000000;
                            std::shared_ptr<SENSORMSG_GNSSDATA> pxGNSSData = std::make_shared<SENSORMSG_GNSSDATA>();
                            pxGNSSData->prn = pxOBSVMData->obsSVInfo[i].prn;
                            pxGNSSData->psr = pxOBSVMData->obsSVInfo[i].psr;
                            pxGNSSData->psrstd = pxOBSVMData->obsSVInfo[i].psr_std;
                            pxGNSSData->CN0 = pxOBSVMData->obsSVInfo[i].CN0;
                            pxGNSSData->type = gnssType;
                            pxGNSSData->freqType = freqType;
                            pxGNSSData->psrRate = pxOBSVMData->obsSVInfo[i].locktime;
                            pxGNSSData->satClk = satECEFLUT[tmpKey].satClk;
                            pxGNSSData->lonoDly = satECEFLUT[tmpKey].lonoDly;
                            pxGNSSData->tropDly = satECEFLUT[tmpKey].tropDly;
                            pxGNSSData->satECEF = satECEFLUT[tmpKey].satECEF;
                            pxGNSSData->pos = Vector3d(pxNMEAGGAData->lat, 
                                                      pxNMEAGGAData->lon,
                                                      pxNMEAGGAData->alt);
                            if (isNeedSensorsLog) {
                                gnssLogFile1 << std::scientific<< std::setprecision(14);
                                gnssLogFile1 << uiSync <<' ' <<(uint32_t)(pxGNSSData->prn)<<' ' <<(uint32_t)(pxGNSSData->type) <<' ' 
                                      <<(uint32_t)(pxGNSSData->freqType) <<  ' ' << pxGNSSData->psr<<  ' ' << pxGNSSData->psrstd<<  ' ' 
                                      << (uint32_t)pxGNSSData->CN0 << ' ' << pxGNSSData->psrRate << ' '
                                      << pxGNSSData->satECEF[0] << ' ' << pxGNSSData->satECEF[1] << ' ' << pxGNSSData->satECEF[2] << ' '
                                      << pxGNSSData->satClk << ' ' << pxGNSSData->lonoDly << ' ' << pxGNSSData->tropDly<< ' '
                                      << pxGNSSData->pos[0] << ' '<< pxGNSSData->pos[1] << ' '<< pxGNSSData->pos[2]  << std::endl;
                            }
                            // gnssPub.publish(pxGNSSData);
                        }
                    }
                    
                }
                if (isNeedSensorsLog) {
                    gnssLogFile1 << std::endl;
                }
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGNSSData = nullptr;
                // gnssPub.publish(pxGNSSData); // denote this frame end

                // invalid SATLUT
                for (size_t i = 0; i < pxSATECEFData->satNum; i++) {
                    uint32_t tmpKey = pxSATECEFData->satECEFInfo[i].prn + ((pxSATECEFData->satECEFInfo[i].sysType) << 8);
                    satECEFLUT[tmpKey].isValid = false;
                }
                // invaild trackedSatsLUT
                for (size_t i = 0; i < pxEUNIPVTSLNData->PRNnums; i++) {
                    uint32_t tmpprn = 0, tmpType = 0;
                    if (parseSYSTYPEandPRN(pxEUNIPVTSLNData->prns[i], tmpprn, tmpType)) {
                        trackSatLUT[tmpprn + (tmpType << 8)] = false;
                    } 
                }
            }
            // complete this gnss packet
            pxOBSVMData = nullptr;
            pxSATECEFData = nullptr;
            pxNMEAGGAData = nullptr;
            pxEUNIPVTSLNData = nullptr;
            uiSync++;
        }
    }
    
}

bool SENSORS::publishThreads()
{
    if (imuSTIM300.isOpen() && gnssUM982.isOpen())
    {
        std::thread imuThread(std::bind(&SENSORS::listenIMU, this));
        imuThread.detach();
        std::thread gnssThread(std::bind(&SENSORS::listenGNSSTC, this));
        gnssThread.detach();
        return true;
    }
    return false;
}

bool SENSORS::config()
{
    return true;
}

bool SENSORS::readFromFile(const std::string& imuFilePath, const std::string& gnssFilePath)
{
    std::ifstream imuFile(imuFilePath);
    std::ifstream gnssFile(gnssFilePath);
    while (imuFile.is_open()||gnssFile.is_open())
    {
        // read imu data
        if (imuFile.is_open()) {
            double tmpVal = 0.0;
            for (size_t i = 0; i < SENSORS_IMU_SAMPLE_FREQ; i++) {
                std::shared_ptr<SENSORMSG_IMUDATA> pxIMUData = std::make_shared<SENSORMSG_IMUDATA>();
                imuFile >>tmpVal;
                pxIMUData->gyro[0] = tmpVal;
                imuFile >>tmpVal;
                pxIMUData->gyro[1] = tmpVal;
                imuFile >>tmpVal;
                pxIMUData->gyro[2] = tmpVal;
                imuFile >>tmpVal;
                pxIMUData->acc[0] = tmpVal;
                imuFile >>tmpVal;
                pxIMUData->acc[1] = tmpVal;
                imuFile >>tmpVal;
                pxIMUData->acc[2] = tmpVal;
                imuPub.publish(pxIMUData);
            }
            if (imuFile.eof()) {
                imuFile.close();
                std::cout << "imu file read done" << std::endl;
            }
        }
        // read gnss data
        if (gnssFile.is_open()) {
            std::string gnssLine;
            while (std::getline(gnssFile, gnssLine)) {
                if (gnssLine.empty()) {
                    std::shared_ptr<SENSORMSG_GNSSDATA> pxGNSSData = nullptr;
                    gnssPub.publish(pxGNSSData);
                    break;
                }
                std::stringstream gnssSs(gnssLine);
                double tmpVal = 0.0;
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGNSSData = std::make_shared<SENSORMSG_GNSSDATA>();
                gnssSs >>tmpVal;
                pxGNSSData->psr = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->psrRate = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->satECEF[0] = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->satECEF[1] = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->satECEF[2] = tmpVal;
                gnssPub.publish(pxGNSSData);
            }
            if (gnssFile.eof()) {
                gnssFile.close();
                std::cout << "gnss file read done" << std::endl;
            }    
        }
    }

    imuFile.close();
    gnssFile.close();
    return true;
}

