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

    luSensors.setLogFile(SENSORSDATAPATH);
    luSensors.setErrLogFile(SENSORSERRLOGFILE);
    luSensors.setIMUTopicName(IMUTOPICNAME);
    luSensors.setGNSSTopicName(GNSSTOPICNAME);

    std::shared_ptr<TCNAV> luNav = std::make_shared<TCNAV>();
    
    // config the navigation primary configuration
    luNav->subscribe(IMUTOPICNAME);
    luNav->subscribe(GNSSTOPICNAME);
    luNav->setRawDataFile(IMUDATAFILE, GNSSDATAFILE);
    luNav->setInsLogFile(TCINSLOGFILE);
    luNav->setErrLogFile(TCINSERRLOGFILE);
    luNav->setRawDataEcho(false);
    
    // listen first
    // std::thread navListenThread(&TCNAV::listen, luNav);
    // navListenThread.detach();

    // publish after
    luSensors.publishThreads();
    // navigation running in the main thread
    // luNav->navRunning();
    while (true)
    {
        /* code */
    }
    
    // luNav->navOnlyReadRawDataTest();
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
