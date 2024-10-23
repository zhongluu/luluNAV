#ifndef _MAIN_
#define _MAIN_

#define IMUTOPICNAME "IMUMSG"
#define GNSSTOPICNAME "GNSSMSG"

#define IMUUARTPORT "/dev/ttyCH9344USB2"
#define GNSSUARTPORT "/dev/ttyCH9344USB1"

#define SENSORSDATAPATH "../data/sensors/"
#define SENSORSERRLOGFILE "../data/sensors/error.txt"

#define IMUDATAFILE "../data/tightcouple/imuRAW.txt"
#define GNSSDATAFILE "../data/tightcouple/gnssRAW.txt"
#define TCINSLOGFILE "../data/tightcouple/tcINSLog.txt"
#define TCINSERRLOGFILE "../data/tightcouple/error.txt"

// #define IMUSIMDATAFILE "../data/SIM/imuData.txt"
// #define GNSSSIMDATAFILE "../data/SIM/gnssData.txt"
// #define TCINSSIMLOGFILE "../data/SIM/tcINSSIMLog.txt"

#define IMUSIMDATAFILE "../data/tightcouple/imuRAW_2217.txt"
#define GNSSSIMDATAFILE "../data/tightcouple/gnssRAW_2217.txt"
#define TCINSSIMLOGFILE "../data/SIM/tcVBINSSIMLog_2217.txt"

// #define USINGNOVELMETHOD

//#define READFROMFILE

#ifndef READFROMFILE
#define GLV_IMU_SAMPLES 125
#else
// #define GLV_IMU_SAMPLES 100
#define GLV_IMU_SAMPLES 250
#endif

#endif