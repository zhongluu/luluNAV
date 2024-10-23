#ifndef _SENSORS_
#define _SENSORS_

#include "Publisher.hpp"
#include "Subscriber.hpp"
#include "IMU.hpp"
#include "GNSS.hpp"
#include <Eigen/Dense>

class SENSORS 
{
private:
    IMU imuSTIM300;
    GNSS gnssUM982;
    Publisher imuPub;
    Publisher gnssPub;
    std::ofstream imuLogFile;
    std::ofstream gnssLogFile1;
    std::ofstream gnssLogFile2;
    std::ofstream errLogFile;
    uint32_t uiSync = 0;
    bool isNeedSensorsLog = false;
    bool parseSYSTYPEandPRN(const uint32_t & orignalPrn, uint32_t& prn, uint32_t& type);
    void listenIMU();
    void listenGNSSTC();

public:
    SENSORS(const std::string& imuPath, const std::string& gnssPath);
    ~SENSORS();
    bool publishThreads();
    bool setIMUTopicName(const std::string& imuTopicName);
    bool setGNSSTopicName(const std::string& gnssTopicName);
    bool setLogFile(const std::string& logFilePath);
    bool setErrLogFile(const std::string& logFilePath);
    bool config();
    bool readFromFile(const std::string& imuFilePath, const std::string& gnssFilePath);
};


#endif