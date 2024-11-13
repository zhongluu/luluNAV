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
#include "tightcouple.h"
#include <iomanip>
#include <ctime>
#include <chrono>

// todo: merge to the configuration of tightcouple
#define IMU_SAMPLE_FREQ GLV_IMU_SAMPLES

#ifdef USINGNOVELMETHOD
TCNAV::TCNAV(/* args */) : isNeedInsLog(false), isNeedRawDataEcho(false), isNeedLogRawData(false),
        isNeedInsEcho(false), isReadFromLog(false), tcINS(), BIVBekf(tcINS)
{

}
#else
TCNAV::TCNAV(/* args */) : isNeedInsLog(false), isNeedRawDataEcho(false), isNeedLogRawData(false),
        isNeedInsEcho(false), isReadFromLog(false), tcINS(), ekf(tcINS)
{

}
#endif
TCNAV::~TCNAV()
{

}

void TCNAV::handleData(const std::string& topicName, std::shared_ptr<void> data)
{
    if (topicName == "IMUMSG") {
        if (data == nullptr) {
            return;
        }
        std::shared_ptr<SENSORMSG_IMUDATA> imuData = std::static_pointer_cast<SENSORMSG_IMUDATA>(data);
        imuDataMtx.lock();
        imuDataQueue.push(imuData);
        imuDataMtx.unlock();
    } else if (topicName == "GNSSMSG") {
        if (data == nullptr)
        {
            gnssDataMtx.lock();
            gnssDataQueue.push(nullptr);
            gnssDataMtx.unlock();
        }else{
            std::shared_ptr<SENSORMSG_GNSSDATA> gnssData = std::static_pointer_cast<SENSORMSG_GNSSDATA>(data);
            gnssDataMtx.lock();
            gnssDataQueue.push(gnssData);
            gnssDataMtx.unlock();
        }
        
        
    } else {
        std::cout << "undefined msg" << std::endl;
    }
}

bool TCNAV::setErrLogFile(const std::string& errLogPath)
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

bool TCNAV::setInsLogFile(const std::string& insLogPath)
{
    if (insLogPath.empty() ) {
        insOutLogFile.close();
        isNeedInsLog = false;
        return true;
    }
    insOutLogFile.close();
    insOutLogFile.open(insLogPath);
    if (insOutLogFile.is_open()) {
        isNeedInsLog = true;
        insOutLogFile << "lcTime pitch roll yaw velE velN velU lat lon height biasGx biasGy biasGz biasAx biasAy biasAz leverx levery leverz dt psrCorr psrRateDrift utc" << std::endl;
    } else {
        return false;
    }
    return true;
}

bool TCNAV::setRawDataFile(const std::string& rawIMUDataPath, const std::string& rawGNSSDataPath)
{
    if (rawIMUDataPath.empty() || rawGNSSDataPath.empty()) {
        rawIMUlogFile.close();
        rawGNSSlogFile.close();
        isNeedLogRawData = false;
        return true;
    }
    rawIMUlogFile.close();
    rawGNSSlogFile.close();
    rawIMUlogFile.open(rawIMUDataPath);
    rawGNSSlogFile.open(rawGNSSDataPath);
    if (rawIMUlogFile.is_open() && rawGNSSlogFile.is_open()) {
        isNeedLogRawData = true;
        rawIMUlogFile << "lcTime Gyro_x Gyro_y Gyro_z Acc_x Acc_y Acc_z debugstamp" << std::endl;
        rawGNSSlogFile << "lcTime PRN type freqType PSR PSRSTD CN0 locktime satECEFx satECEFy satECEFz satclkDly lonoDly tropDly lat lon alt debugstamp utc " << std::endl;
    } else {
        return false;
    }
    return true;
}

bool TCNAV::setRawDataEcho(bool flag)
{
    isNeedRawDataEcho = flag;
    return true;
}

bool TCNAV::setInsEcho(bool flag)
{
    isNeedInsEcho = flag;
    return true;
}

bool TCNAV::setReadFromLog(bool flag)
{
    isReadFromLog = flag;
    return true;
}

void TCNAV::navOnlyReadRawDataTest()
{
    uint32_t debugTimestamp = 0;
    while (true)
    {
        if(gnssDataMtx.try_lock()) {
            if(!gnssDataQueue.empty()) {
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGnssData = gnssDataQueue.front();
                gnssDataQueue.pop();
                gnssDataMtx.unlock();
                if (pxGnssData != nullptr) {
                    if (isNeedLogRawData) {
                        rawGNSSlogFile  << std::scientific<< std::setprecision(14);
                        rawGNSSlogFile << std::time(nullptr) <<' ' <<(pxGnssData->prn) <<' ' <<(uint32_t)(pxGnssData->type) <<' ' 
                                          <<(uint32_t)(pxGnssData->freqType) <<  ' ' << pxGnssData->psr <<  ' ' << pxGnssData->psrstd<<  ' ' 
                                          << (uint32_t)pxGnssData->CN0 << ' ' << pxGnssData->psrRate << ' '
                                          << pxGnssData->satECEF[0] << ' ' << pxGnssData->satECEF[1] << ' ' << pxGnssData->satECEF[2] << ' '
                                          << pxGnssData->satClk << ' ' << pxGnssData->lonoDly << ' ' << pxGnssData->tropDly <<' '
                                          << pxGnssData->pos[0] << ' '<< pxGnssData->pos[1] << ' '<< pxGnssData->pos[2]  << std::endl;
                    }
                    if (isNeedRawDataEcho) {
                        std::cout << std::time(nullptr) << ' ' << debugTimestamp << ' ' << pxGnssData->psr << ' ' << pxGnssData->psrRate << ' '
                                      << pxGnssData->satECEF[0] << ' ' << pxGnssData->satECEF[1] << ' ' << pxGnssData->satECEF[2] << std::endl;
                    }
                } else {
                    debugTimestamp++;
                    if (isNeedLogRawData) {
                        rawGNSSlogFile << std::endl;
                    }
                    if (isNeedRawDataEcho) {
                        std::cout << std::endl;
                    }
                }
            } else {
                gnssDataMtx.unlock();
            }
        }
        
        if (imuDataMtx.try_lock()) {
            if (!imuDataQueue.empty()) {
                std::shared_ptr<SENSORMSG_IMUDATA> pxImuData = imuDataQueue.front();
                imuDataQueue.pop();
                imuDataMtx.unlock();
                if (pxImuData != nullptr) {
                    if (isNeedLogRawData) {
                        rawIMUlogFile << std::scientific<< std::setprecision(12);
                        rawIMUlogFile << std::time(nullptr) << ' ' << debugTimestamp << ' ' << pxImuData->gyro[0] << ' ' << pxImuData->gyro[1] << ' ' << pxImuData->gyro[2] << ' '
                                      << pxImuData->acc[0] << ' ' << pxImuData->acc[1] << ' ' << pxImuData->acc[2] << std::endl;
                    }
                    if (isNeedRawDataEcho) {
                        std::cout << std::time(nullptr) << ' ' << debugTimestamp << ' '  << pxImuData->gyro[0] << ' ' << pxImuData->gyro[1] << ' ' << pxImuData->gyro[2] << ' '
                                      << pxImuData->acc[0] << ' ' << pxImuData->acc[1] << ' ' << pxImuData->acc[2] << std::endl;
                    }
                }
            } else {
                imuDataMtx.unlock();
            }
            
        }
    }
}

void TCNAV::navRunning()
{
    if (isReadFromLog) {
        _gnssWaitTime = 10000000;
        _tcNavState = TCNAVRunState::init;
        tcINS.insConfig("../config/insSIMconfigB.txt");
    } else {
        tcINS.insConfig("../config/insEXconfig.txt");
    }
#ifdef USINGNOVELMETHOD
    BIVBekf.init();
#else
    ekf.init();
#endif

#ifdef EVALUATETIME
    auto startEvaTime = std::chrono::steady_clock::now();
    auto endEvaTime = std::chrono::steady_clock::now();
    
    elipseTimeLog.open("../data/tightcouple/time.txt");
    if (elipseTimeLog.is_open()) {
        elipseTimeLog << "Time Log File" << std::endl;
    }
#endif
    const TCINSSTATE insState = tcINS.getInsState();
    std::time_t timePureInertiaWatchDog;
    auto startGPSWaitTime = std::chrono::steady_clock::now();
    auto endGPSWaitTime = std::chrono::steady_clock::now();
    uint32_t debugTimestamp = 0;
    double utcTime = 0.0;
    while (true)
    {
        switch (_tcNavState)
        {
        case TCNAVRunState::dropDrity:
            gnssDataMtx.lock();
            if (!gnssDataQueue.empty()) {
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGnssData = gnssDataQueue.front();
                gnssDataQueue.pop();
                gnssDataMtx.unlock();
                if (pxGnssData == nullptr) {
                    debugTimestamp++;
                    if (debugTimestamp >= 30) {
                        _tcNavState = TCNAVRunState::sync;
                        break;
                    } 
                }
            } else {
                gnssDataMtx.unlock();
            }
            imuDataMtx.lock();
            if (!imuDataQueue.empty()) {
                imuDataQueue.pop();
            }
            imuDataMtx.unlock(); 
            break;
        case TCNAVRunState::sync:
            gnssDataMtx.lock();
            if (!gnssDataQueue.empty()) {
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGnssData = gnssDataQueue.front();
                gnssDataMtx.unlock();
                if (pxGnssData != nullptr) {
                    imuDataMtx.lock();
                    while (!imuDataQueue.empty()) {
                        std::shared_ptr<SENSORMSG_IMUDATA> pxImuData = imuDataQueue.front();
                        if (pxGnssData->gnssTime == pxImuData->imuTime) {
                            Vector3d pos;
                            int deg = static_cast<int>(pxGnssData->pos[0] / 100);
                            double minsec = pxGnssData->pos[0] - deg * 100.0;
                            pos[0] = ((deg * 1.0) + (minsec / 60.0)) * navGlv::deg2rad;
                            deg = static_cast<int>(pxGnssData->pos[1] / 100);
                            minsec = pxGnssData->pos[1] - deg * 100.0;
                            pos[1] = ((deg * 1.0) + (minsec / 60.0)) * navGlv::deg2rad;
                            pos[2] = pxGnssData->pos[2];
                            tcINS.setPos(pos);
                            _tcNavState = TCNAVRunState::init;
                            debugTimestamp = 0;
                            break;
                            // imuDataMtx.unlock();
                        } else {
                            imuDataQueue.pop();
                            imuDataMtx.unlock();
                        }
                    } 
                    imuDataMtx.unlock();
                }
            } else {
                gnssDataMtx.unlock();
            }
            break;
        case TCNAVRunState::init:
        // waiting for the first valid measurement 
        // clear imu cache
            gnssDataMtx.lock();
            if (!gnssDataQueue.empty()) {
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGnssData = gnssDataQueue.front();
                gnssDataQueue.pop();
                gnssDataMtx.unlock();
                startGPSWaitTime = std::chrono::steady_clock::now();
                if (pxGnssData == nullptr) {
                    if (isNeedLogRawData) {
                        rawGNSSlogFile << std::endl;
                    }
                    if (isNeedInsEcho) {
                        std::cout << "TCINS Running..." << std::endl;
                        errLogFile << std::time(nullptr) << "TCINS Running..." << std::endl;
                    }
                    // successful receive the first gnss info;
#ifdef USINGNOVELMETHOD
                    BIVBekf.init();
#else
                    ekf.init();
#endif
                    _gnssWaitTime = 1000;
                    _tcNavState = TCNAVRunState::timeUpdate;
                    timePureInertiaWatchDog = std::time(nullptr) + _pureInertiaWaitTime;
                } else {
                    // imuDataMtx.lock();
                    // while (!imuDataQueue.empty()) {
                    //     imuDataQueue.pop();
                    // }
                    // imuDataMtx.unlock(); 
                }
            } else {
                gnssDataMtx.unlock();
            }
            // wait gnss time out
            if (_tcNavState == TCNAVRunState::init) {
                endGPSWaitTime = std::chrono::steady_clock::now();
                int64_t elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endGPSWaitTime - startGPSWaitTime).count();
                if (elapsed > _gnssWaitTime) {
                    imuDataMtx.lock();
                    while (!imuDataQueue.empty()) {
                        imuDataQueue.pop();
                    }
                    imuDataMtx.unlock(); 
                    _tcNavState = TCNAVRunState::dropDrity;
                    errLogFile << std::time(nullptr) << ':' << "wait GNSS time out in Initial State" << std::endl;
                }
            }
            break;
        case TCNAVRunState::timeUpdate:
            if (imuDataMtx.try_lock()) {
                if (!imuDataQueue.empty()) {
                    std::shared_ptr<SENSORMSG_IMUDATA> pxImuData = imuDataQueue.front();
                    imuDataQueue.pop();
                    imuDataMtx.unlock();
                    if (pxImuData != nullptr) {
                        if (isNeedLogRawData) {
                            rawIMUlogFile << std::scientific<< std::setprecision(12);
                            rawIMUlogFile << pxImuData->imuTime  << ' ' << pxImuData->gyro[0] << ' ' << pxImuData->gyro[1] << ' ' << pxImuData->gyro[2] << ' '
                                          << pxImuData->acc[0] << ' ' << pxImuData->acc[1] << ' ' << pxImuData->acc[2] << ' ' << debugTimestamp << ' '  << std::endl;
                        }
                        if (isNeedRawDataEcho) {
                            std::cout << std::time(nullptr) << ' ' << pxImuData->gyro[0] << ' ' << pxImuData->gyro[1] << ' ' << pxImuData->gyro[2] << ' '
                                          << pxImuData->acc[0] << ' ' << pxImuData->acc[1] << ' ' << pxImuData->acc[2] << std::endl;
                        }
                        tcINS.insUpdate(pxImuData->gyro, pxImuData->acc);
#ifdef USINGNOVELMETHOD
                        BIVBekf.timeUpdate();
#else
                        ekf.timeUpdate();
#endif
                        _imuDataCount++;
                    }
                } else {
                    imuDataMtx.unlock();
                }
                if (_imuDataCount == IMU_SAMPLE_FREQ) {
                    _imuDataCount = 0;
                    timePureInertiaWatchDog = std::time(nullptr) + _pureInertiaWaitTime;
#ifdef USINGNOVELMETHOD
                    BIVBekf.seqBIVBEKFTCNHC();
#else
                    ekf.seqEKFTCNHC();
#endif
                    _tcNavState = TCNAVRunState::measurementUpdate;
#ifdef EVALUATETIME
                    startEvaTime = std::chrono::steady_clock::now();
#endif
                    debugTimestamp++;
                    startGPSWaitTime = std::chrono::steady_clock::now();
                }
            }
            if (timePureInertiaWatchDog < std::time(nullptr)) {
                errLogFile << std::time(nullptr) << ':' << "IMU Pure inertial time out" << std::endl;
                _tcNavState = TCNAVRunState::init;
            }
            break;
        case TCNAVRunState::measurementUpdate:
            if (gnssDataMtx.try_lock()) {
                if (!gnssDataQueue.empty()) {
                    std::shared_ptr<SENSORMSG_GNSSDATA> pxGnssData = gnssDataQueue.front();
                    gnssDataQueue.pop();
                    gnssDataMtx.unlock();
                    if (pxGnssData != nullptr) {
#ifdef USINGNOVELMETHOD
                        BIVBekf.seqBIVBMeasurementUpdate(pxGnssData->prn*256 + pxGnssData->freqType, pxGnssData->psr + pxGnssData->satClk + pxGnssData->lonoDly + pxGnssData->tropDly, pxGnssData->satECEF);
#else
                        ekf.seqMeasurementUpdate(pxGnssData->psr + pxGnssData->satClk + pxGnssData->lonoDly + pxGnssData->tropDly, pxGnssData->satECEF);
#endif
                        utcTime = pxGnssData->utcTime;
                        if (isNeedLogRawData) {
                            rawGNSSlogFile  << std::scientific<< std::setprecision(14);
                            rawGNSSlogFile << pxGnssData->gnssTime <<' ' <<(pxGnssData->prn) <<' ' <<(uint32_t)(pxGnssData->type) <<' ' 
                                          <<(uint32_t)(pxGnssData->freqType) <<  ' ' << pxGnssData->psr <<  ' ' << pxGnssData->psrstd<<  ' ' 
                                          << (uint32_t)pxGnssData->CN0 << ' ' << pxGnssData->psrRate << ' '
                                          << pxGnssData->satECEF[0] << ' ' << pxGnssData->satECEF[1] << ' ' << pxGnssData->satECEF[2] << ' '
                                          << pxGnssData->satClk << ' ' << pxGnssData->lonoDly << ' ' << pxGnssData->tropDly <<' '
                                          << pxGnssData->pos[0] << ' '<< pxGnssData->pos[1] << ' '<< pxGnssData->pos[2] << ' ' << debugTimestamp << ' ' << utcTime << ' '<< std::endl;
                        }
                        if (isNeedRawDataEcho) {
                            std::cout  << std::time(nullptr) <<' ' <<(int)(pxGnssData->prn) <<  ' '  << pxGnssData->psr << ' ' << pxGnssData->psrRate << ' '
                                          << pxGnssData->satECEF[0] << ' ' << pxGnssData->satECEF[1] << ' ' << pxGnssData->satECEF[2] << std::endl;
                        }
                        startGPSWaitTime = std::chrono::steady_clock::now();
                    } else {
                        // gnssDataMtx.unlock();
                        if (isNeedLogRawData) {
                            rawGNSSlogFile << std::endl;
                        }
                        if (isNeedRawDataEcho) {
                            std::cout << std::endl;
                        }
#ifdef USINGNOVELMETHOD
                        BIVBekf.seqBIVBUpdateLUT();
#endif
#ifdef EVALUATETIME
                        endEvaTime = std::chrono::steady_clock::now();
                        std::chrono::duration<double> duration = 
                            std::chrono::duration_cast<std::chrono::duration<double>>(endEvaTime - startEvaTime);
                        elipseTimeLog << "Elapsed time: " << duration.count() << " seconds"<< std::endl;
                        // elipseTimeLog << "Elapsed time: "  << " seconds" << std::endl;
#endif
                        _tcNavState = TCNAVRunState::navEchoAndLog;
                        break;
                    }
                } else {
                    gnssDataMtx.unlock();
                }
            }
            // wait gnss time out
            if (_tcNavState == TCNAVRunState::measurementUpdate) {
                endGPSWaitTime = std::chrono::steady_clock::now();
                int64_t elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endGPSWaitTime - startGPSWaitTime).count();
                if (elapsed > _gnssWaitTime) {
                    _tcNavState = TCNAVRunState::navEchoAndLog;
                    errLogFile << std::time(nullptr) << ':' << "GNSS time out in measurement update" << std::endl;
                }
            }
            break;
        case TCNAVRunState::navEchoAndLog:
            if (isNeedInsEcho) {
                std::cout << std::setprecision(12);
                std::cout << std::time(nullptr) << ' '
                          << insState.atti[0] * navGlv::rad2deg << ' ' << insState.atti[1] * navGlv::rad2deg << ' ' << insState.atti[2] * navGlv::rad2deg << ' ' 
                          << insState.vn[0] << ' ' << insState.vn[1] << ' ' << insState.vn[2] << ' '
                          << insState.pos[0] << ' ' << insState.pos[1] << ' ' << insState.pos[2] << ' '
                          << insState.biasG[0] << ' ' << insState.biasG[1] << ' ' << insState.biasG[2] << ' '
                          << insState.biasA[0] << ' ' << insState.biasA[1] << ' ' << insState.biasA[2] << ' '
                          << insState.lever[0] << ' ' << insState.lever[1] << ' ' << insState.lever[2] << ' '
                          << insState.dt << ' ' << insState.ccorr << ' ' << insState.cdrift << ' ' <<utcTime<< ' '<< std::endl;
            }
            if (isNeedInsLog) {
                insOutLogFile << std::scientific << std::setprecision(12);
                insOutLogFile <<std::time(nullptr) << ' '
                              << insState.atti[0] * navGlv::rad2deg << ' ' << insState.atti[1] * navGlv::rad2deg << ' ' << insState.atti[2] * navGlv::rad2deg << ' ' 
                              << insState.vn[0] << ' ' << insState.vn[1] << ' ' << insState.vn[2] << ' '
                              << insState.pos[0] << ' ' << insState.pos[1] << ' ' << insState.pos[2] << ' '
                              << insState.biasG[0] << ' ' << insState.biasG[1] << ' ' << insState.biasG[2] << ' '
                              << insState.biasA[0] << ' ' << insState.biasA[1] << ' ' << insState.biasA[2] << ' '
                              << insState.lever[0] << ' ' << insState.lever[1] << ' ' << insState.lever[2] << ' '
                              << insState.dt << ' ' << insState.ccorr << ' ' << insState.cdrift << ' ' <<utcTime << ' ' <<std::endl;
            }
            _tcNavState = TCNAVRunState::timeUpdate;
            break;
        default:
            break;
        }
    }
}

void TCNAV::readRawDataFromFile(const std::string& imuFilePath, const std::string& gnssFilePath)
{
    std::ifstream imuFile(imuFilePath);
    std::ifstream gnssFile(gnssFilePath);

    if (imuFile.is_open()&&gnssFile.is_open())
    {
        std::string tmpLine;
        std::getline(imuFile, tmpLine);
        // std::cout << tmpLine;
        std::getline(gnssFile, tmpLine);
        // std::cout << tmpLine;
    }
    // uint32_t tmpcount = 0;

    while (imuFile.is_open()||gnssFile.is_open())
    {
        // read imu data
        if (imuFile.is_open()) {
            double tmpVal = 0.0;
            for (size_t i = 0; i < IMU_SAMPLE_FREQ; i++) {
                std::shared_ptr<SENSORMSG_IMUDATA> pxIMUData = std::make_shared<SENSORMSG_IMUDATA>();
                imuFile >>tmpVal; imuFile >>tmpVal;
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
                imuFile >>tmpVal;
                // imuPub.publish(pxIMUData);
                imuDataMtx.lock();
                // std::cout << pxIMUData->gyro[0] << ' ' << pxIMUData->gyro[1] << ' ' << pxIMUData->gyro[2] << ' '
                //                       << pxIMUData->acc[0] << ' ' << pxIMUData->acc[1] << ' ' << pxIMUData->acc[2] << std::endl;
                imuDataQueue.push(pxIMUData);
                imuDataMtx.unlock();
                
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
                // if (gnssLine.empty()) {
                if (gnssLine=="\r"||gnssLine==""||gnssLine=="\n") {
                    std::shared_ptr<SENSORMSG_GNSSDATA> pxGNSSData = nullptr;
                    // gnssPub.publish(pxGNSSData);
                    gnssDataMtx.lock();
                    gnssDataQueue.push(pxGNSSData);
                    gnssDataMtx.unlock();
                    // std::cout<<"entering";
                    // tmpcount++;
                    // std::cout<<tmpcount <<std::endl;
                    break;
                }
                // std::cout<<gnssLine;
                std::stringstream gnssSs(gnssLine);
                double tmpVal = 0.0;
                std::shared_ptr<SENSORMSG_GNSSDATA> pxGNSSData = std::make_shared<SENSORMSG_GNSSDATA>();
                gnssSs >>tmpVal; gnssSs >>tmpVal; 
                pxGNSSData->prn = tmpVal; 
                gnssSs >>tmpVal; gnssSs >>tmpVal; 
                pxGNSSData->freqType = tmpVal; 
                gnssSs >>tmpVal;
                pxGNSSData->psr = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->psrRate = tmpVal;
                gnssSs >>tmpVal; gnssSs >>tmpVal; gnssSs >>tmpVal;
                pxGNSSData->satECEF[0] = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->satECEF[1] = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->satECEF[2] = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->satClk = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->lonoDly = tmpVal;
                gnssSs >>tmpVal;
                pxGNSSData->tropDly = tmpVal;
                gnssDataMtx.lock();
                gnssDataQueue.push(pxGNSSData);
                gnssDataMtx.unlock();
            }
            if (gnssFile.eof()) {
                gnssFile.close();
                std::cout << "gnss file read done" << std::endl;
            }    
        }
    }

    imuFile.close();
    gnssFile.close();
}