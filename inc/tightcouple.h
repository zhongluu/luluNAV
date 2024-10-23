#ifndef _TIGHTCOUPLE_
#define _TIGHTCOUPLE_
#include "Subscriber.hpp"
#include "BASEElement.h"
#include "sensorsMsg.hpp"
#include <mutex>
#include "insTC.h"
#include <iostream>

class TCNAV : public Subscriber
{
private:

    enum class TCNAVRunState {
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
    TCNAVRunState _tcNavState = TCNAVRunState::init;
    uint32_t _imuDataCount{0};
    uint32_t _pureInertiaWaitTime{1200}; //default - 1200 (s)
    int64_t _gnssWaitTime{500};  //default - 30 (ms) in measurement update state
    std::ofstream insOutLogFile;
    std::ofstream errLogFile;
    std::ofstream rawIMUlogFile;
    std::ofstream rawGNSSlogFile;
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