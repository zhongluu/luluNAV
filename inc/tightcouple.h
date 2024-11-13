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
#ifndef _TIGHTCOUPLE_
#define _TIGHTCOUPLE_
#include "Subscriber.hpp"
#include "BASEElement.h"
#include "main.hpp"
#include "sensorsMsg.hpp"
#include <mutex>
#include "insTC.h"
#include <iostream>

class TCNAV : public Subscriber
{
private:

    enum class TCNAVRunState {
        dropDrity,
        sync,
		init,
		timeUpdate,
		measurementUpdate,
        navEchoAndLog
	};

    TCINS tcINS;

#ifdef USINGNOVELMETHOD
    TCINS::seqBIVBEKF BIVBekf;
#else
    TCINS::seqEKF ekf;
#endif

    
    bool isNeedInsLog; // default false
    bool isNeedLogRawData; // default false
    bool isNeedInsEcho; // default false
    bool isNeedRawDataEcho; // default false
    bool isReadFromLog;
    TCNAVRunState _tcNavState = TCNAVRunState::dropDrity;
    uint32_t _imuDataCount{0};
    uint32_t _pureInertiaWaitTime{1200}; //default - 1200 (s)
    int64_t _gnssWaitTime{1000};  //default - 30 (ms) in measurement update state
    std::ofstream insOutLogFile;
    std::ofstream errLogFile;
    std::ofstream rawIMUlogFile;
    std::ofstream rawGNSSlogFile;
    std::ofstream elipseTimeLog;
    std::mutex imuDataMtx;
    std::queue<std::shared_ptr<SENSORMSG_IMUDATA>> imuDataQueue;
    std::mutex gnssDataMtx;
    std::queue<std::shared_ptr<SENSORMSG_GNSSDATA>> gnssDataQueue;
public:
    TCNAV(/* args */);
    ~TCNAV();
    bool setErrLogFile(const std::string& errLogPath);
    bool setInsLogFile(const std::string& insLogPath);
    bool setRawDataFile(const std::string& rawIMUDataPath, const std::string& rawGNSSDataPath);
    bool setRawDataEcho(bool flag);
    bool setInsEcho(bool flag);
    bool setReadFromLog(bool flag);
    void handleData(const std::string& topicName, std::shared_ptr<void> data) override;
    void readRawDataFromFile(const std::string& imuFilePath, const std::string& gnssFilePath);
    void navOnlyReadRawDataTest();
    void navRunning();
};

#endif