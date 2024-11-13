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