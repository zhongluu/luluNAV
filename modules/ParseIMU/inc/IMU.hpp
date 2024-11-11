#ifndef _PARSEIMU_
#define _PARSEIMU_

#include "serial.hpp"
#include <memory>
#include <iostream>
#include <fstream>
#include <mutex>
// #include <thread>
#include <cstdint>
#include "RingBuffer.hpp"



#define STIM300_READ_BUFFER_SIZE 42
#define STIM300_FRAME_HEAD 0xA5

#define IMU_RECV_BUFFER_SIZE (STIM300_READ_BUFFER_SIZE * 20)

typedef struct tagIMUData
{
    double gx,gy,gz; // gyro. output (deg/s)
    double ax,ay,az; // acc. output (g)
    float tgx,tgy,tgz; // gyro. temperature (C)
    float tax,tay,taz; // acc. temperature (C)
} IMUData;



class IMU
{
private:

    enum class IMUDecodeState {
		uninit,
		got_frameHead,
		check_CRC
	};

    // private data
    SerialPort imuPort;
    std::ofstream logFile;
    bool isNeedLog; // default false
    bool isNeedEcho; // default false
    bool isPortOpen;
    bool isOldVal; // default false
    RingBuffer<uint8_t> _buf;
    uint8_t _imuFrame[STIM300_READ_BUFFER_SIZE];
    uint32_t _revWaitTime; // s , default 5 (min) -- 300 (s)
    IMUDecodeState _decode_state{IMUDecodeState::uninit};
    IMUData _imuData{0};
    std::mutex _mtx;

    // private method
    bool CRCcheck();
    bool parseSTIM300(); // TODO: factory - product pattern
    bool handledMessage();
 
public:
    IMU(const std::string& imuSerialPath, int baudRate = 460800);
    ~IMU();
    bool setLogFile(const std::string& logPath);
    bool setEcho(bool flag);
    bool isOpen();
    bool changeWaitTime(uint32_t delayInS);

    // return false when fail to get imuData or imuData is old 
    bool getIMUData(IMUData& imuData);

    // occupying at a thread, need call getIMUData() at orther thread to get data
    // if not success receive complete imu frame in revWaitTime return false
    int listenInfLoop(); 

    // single time, real time 
    // if not success receive complete imu frame in revWaitTime return false
    int listen(IMUData& imuData); // success return 1 
};

#endif
