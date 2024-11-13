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
#include <iostream>
#include <thread>
#include <cstdint>
#include <cstring> 

#include "main.hpp"

#include "sensors.hpp"
#include "tightcouple.h"



int main()
{
#ifndef READFROMFILE
    SENSORS luSensors(IMUUARTPORT, GNSSUARTPORT);

    // luSensors.setLogFile(SENSORSDATAPATH);
    luSensors.setErrLogFile(SENSORSERRLOGFILE);
    luSensors.setIMUTopicName(IMUTOPICNAME);
    luSensors.setGNSSTopicName(GNSSTOPICNAME);

    std::shared_ptr<TCNAV> luNav = std::make_shared<TCNAV>();
    
    // config the navigation primary configuration
    luNav->subscribe(IMUTOPICNAME);
    luNav->subscribe(GNSSTOPICNAME);
    luNav->setInsEcho(true);
    luNav->setRawDataFile(IMUDATAFILE, GNSSDATAFILE);
    luNav->setInsLogFile(TCINSLOGFILE);
    luNav->setErrLogFile(TCINSERRLOGFILE);
    luNav->setRawDataEcho(false);
    
    // listen first
    std::thread navListenThread(&TCNAV::listen, luNav);
    navListenThread.detach();

    // publish after
    luSensors.publishThreads();
    // navigation running in the main thread
    luNav->navRunning();

#else
    std::shared_ptr<TCNAV> luNav = std::make_shared<TCNAV>();
    luNav->setInsEcho(true);
    luNav->setReadFromLog(true);
    luNav->setInsLogFile(TCINSSIMLOGFILE);
    std::thread navThread(&TCNAV::navRunning, luNav);
    luNav->readRawDataFromFile(IMUSIMDATAFILE, GNSSSIMDATAFILE);
    navThread.join();
#endif
    return 0;
}
