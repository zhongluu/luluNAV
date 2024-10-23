#include "IMU.hpp"
#include <ctime>

IMU::IMU(const std::string& imuSerialPath, int baudRate) : imuPort(imuSerialPath, baudRate), isNeedLog(false), isNeedEcho(false), 
        _revWaitTime(300), _buf(IMU_RECV_BUFFER_SIZE), isOldVal(false), isPortOpen(false)
{
    if(imuPort.openPort()) {
        isPortOpen = true;
    }
}

IMU::~IMU()
{
    imuPort.closePort();
    logFile.close();
}

bool IMU::CRCcheck()
{
    uint32_t checkSum = 0xffffffff;
    uint32_t crcData = 0;
    crcData = (_imuFrame[STIM300_READ_BUFFER_SIZE - 4] << 24) | (_imuFrame[STIM300_READ_BUFFER_SIZE - 3] << 16) | (_imuFrame[STIM300_READ_BUFFER_SIZE - 2] << 8) | (_imuFrame[STIM300_READ_BUFFER_SIZE - 1]);
    _imuFrame[STIM300_READ_BUFFER_SIZE - 4] = 0x00; _imuFrame[STIM300_READ_BUFFER_SIZE - 3] = 0x00;

    for (size_t i = 0; i < (STIM300_READ_BUFFER_SIZE - 2); i++)
    {
        checkSum ^= (uint32_t)(_imuFrame[i]) << 24; 
        for (size_t j = 0; j < 8; j++)
        {
            if (checkSum & 0x80000000)
            {
                checkSum = (checkSum << 1) ^ 0x04C11DB7;
            } else {
                checkSum <<= 1;
            }
            
        }
    }
    
    if (crcData != checkSum)
    {
        return false;
    }
    return true;
}

bool IMU::parseSTIM300()
{
    uint8_t curB = _buf.front();
    switch (_decode_state)
    {
    case IMUDecodeState::uninit:
        if (curB == STIM300_FRAME_HEAD) {
            _decode_state = IMUDecodeState::got_frameHead;
        } else {
            _buf.popOut();
        }
        break;
    case IMUDecodeState::got_frameHead:
        if (_buf.readN(_imuFrame, STIM300_READ_BUFFER_SIZE) == true) {
            _decode_state = IMUDecodeState::check_CRC;
        }
        break;
    case IMUDecodeState::check_CRC:
        if (CRCcheck()) {
            _buf.popNOut(STIM300_READ_BUFFER_SIZE);
            _decode_state = IMUDecodeState::uninit;
            return true;
        } else {
            _buf.popOut();
            _decode_state = IMUDecodeState::uninit;
        }
        break;
    }
    return false;
}

bool IMU::handledMessage()
{
    int data = 0;
    data = (_imuFrame[1] << 24) | (_imuFrame[2] << 16) | (_imuFrame[3] << 8) | 0;
    _imuData.gx = (double)(data * 1.0 / 4194304);
    data = (_imuFrame[4] << 24) | (_imuFrame[5] << 16) | (_imuFrame[6] << 8) | 0;
    _imuData.gy = (double)(data * 1.0 / 4194304);
    data = (_imuFrame[7] << 24) | (_imuFrame[8] << 16) | (_imuFrame[9] << 8) | 0;
    _imuData.gz = (double)(data * 1.0 / 4194304); // 2^(14 + 8)

    data = (_imuFrame[11] << 24) | (_imuFrame[12] << 16) | (_imuFrame[13] << 8) | 0;
    _imuData.ax = (double)(data * 1.0 / 16777216);
    // _imuData.ax = (double)(data * 1.0 / 134217728);
    data = (_imuFrame[14] << 24) | (_imuFrame[15] << 16) | (_imuFrame[16] << 8) | 0;
    _imuData.ay = (double)(data * 1.0 / 16777216);
    // _imuData.ay = (double)(data * 1.0 / 134217728);
    data = (_imuFrame[17] << 24) | (_imuFrame[18] << 16) | (_imuFrame[19] << 8) | 0;
    _imuData.az = (double)(data * 1.0 / 16777216); // 2^(16 + 8) -- 80g
    // _imuData.az = (double)(data * 1.0 / 134217728); // 2^(19 + 8) -- 10g

    data = (_imuFrame[21] << 24) | (_imuFrame[22] << 16); 
    _imuData.tgx = (double)(data * 1.0 / 16777216);
    data = (_imuFrame[23] << 24) | (_imuFrame[24] << 16); 
    _imuData.tgy = (double)(data * 1.0 / 16777216);
    data = (_imuFrame[25] << 24) | (_imuFrame[26] << 16); 
    _imuData.tgz = (double)(data * 1.0 / 16777216); //  2^(8 + 16)

    data = (_imuFrame[28] << 24) | (_imuFrame[29] << 16); 
    _imuData.tax = (double)(data * 1.0 / 16777216);
    data = (_imuFrame[30] << 24) | (_imuFrame[31] << 16); 
    _imuData.tay = (double)(data * 1.0 / 16777216);
    data = (_imuFrame[32] << 24) | (_imuFrame[33]);
    _imuData.taz = (double)(data * 1.0 / 16777216);

    return true;
}

bool IMU::setLogFile(const std::string& logPath)
{
    if (logPath.empty())
    {
        logFile.close();
        isNeedLog = false;
        return true;
    }
    logFile.close();
    logFile.open(logPath);
    if (logFile.is_open()) {
        isNeedLog = true;
        logFile << "Gyro_x Gyro_y Gyro_z Acc_x Acc_y Acc_z TempGx TempGy TempGz TempAx TempAy TempAz"<< std::endl;
    } else {
        return false;
    }
    return true;
}

bool IMU::setEcho(bool flag)
{
    isNeedEcho = flag;
    return true;
}

bool IMU::getIMUData(IMUData& imuData)
{
    bool ret = false;
    if (_mtx.try_lock())
    {
        imuData = _imuData;
        _mtx.unlock();
        if (isOldVal) {
            ret = false;
        } else {
            isOldVal = true;
            ret = true;
        }
    }
    return ret;
}

int IMU::listenInfLoop()
{
    uint8_t imuRecvChar;
    bool ret = false;
    
    std::time_t timeEndPoint = std::time(nullptr) + _revWaitTime;
    while (true)
    {
        int iRet = imuPort.readData((char *)&imuRecvChar, sizeof(imuRecvChar));

        if (iRet < 0) {
            /* code */
            return iRet;

        } else if (iRet != 0) {
            ret = _buf.pushIn(imuRecvChar);
            if (ret != true) {
                std::cout<< "full"<< std::endl;
                return -1;
            }
        } 

        if (_buf.curLen() > STIM300_READ_BUFFER_SIZE) {
            ret = parseSTIM300();
            if (ret == true) {
                _mtx.lock();
                ret = handledMessage();
                isOldVal = false;
                _mtx.unlock();
                if (ret == true) {
                    // 
                    if (isNeedLog)
                    {
                        logFile <<_imuData.gx <<' '<<_imuData.gy <<' '<<_imuData.gz <<' '
                                <<_imuData.ax <<' '<<_imuData.ay <<' '<<_imuData.az <<' '
                                <<_imuData.tgx<<' '<<_imuData.tgy<<' '<<_imuData.tgz<<' '
                                <<_imuData.tax<<' '<<_imuData.tay<<' '<<_imuData.taz<<"\r\n";
                    }
                    if (isNeedEcho)
                    {
                        std::cout <<_imuData.gx <<' '<<_imuData.gy <<' '<<_imuData.gz <<' '
                                  <<_imuData.ax <<' '<<_imuData.ay <<' '<<_imuData.az <<' '
                                  <<_imuData.tgx<<' '<<_imuData.tgy<<' '<<_imuData.tgz<<' '
                                  <<_imuData.tax<<' '<<_imuData.tay<<' '<<_imuData.taz<<std::endl;
                    }
                }
                // feed watchdog
                timeEndPoint = std::time(nullptr) + _revWaitTime;
            }
            
        }

        if (timeEndPoint < std::time(nullptr) ) {
            // imuPort.closePort();
            return -1;
        }
        
    }
    
}

int IMU::listen(IMUData& imuData)
{
    uint8_t imuRecvChar;
    bool ret = false;
    
    std::time_t timeEndPoint = std::time(nullptr) + _revWaitTime;
    while (true)
    {
        int iRet = imuPort.readData((char *)&imuRecvChar, sizeof(imuRecvChar));

        if (iRet < 0) {
            /* code */
            return iRet;

        } else if (iRet != 0) {
            ret = _buf.pushIn(imuRecvChar);
            if (ret != true) {
                std::cout<< "full"<< std::endl;
                return -1;
            }
        } 

        if (_buf.curLen() > STIM300_READ_BUFFER_SIZE) {
            ret = parseSTIM300();
            if (ret == true) {
                ret = handledMessage();
                if (ret == true) {
                    if (isNeedLog) {
                        logFile <<_imuData.gx <<' '<<_imuData.gy <<' '<<_imuData.gz <<' '
                                <<_imuData.ax <<' '<<_imuData.ay <<' '<<_imuData.az <<' '
                                <<_imuData.tgx<<' '<<_imuData.tgy<<' '<<_imuData.tgz<<' '
                                <<_imuData.tax<<' '<<_imuData.tay<<' '<<_imuData.taz<<"\r\n";
                    }
                    if (isNeedEcho) {
                        std::cout <<_imuData.gx <<' '<<_imuData.gy <<' '<<_imuData.gz <<' '
                                  <<_imuData.ax <<' '<<_imuData.ay <<' '<<_imuData.az <<' '
                                  <<_imuData.tgx<<' '<<_imuData.tgy<<' '<<_imuData.tgz<<' '
                                  <<_imuData.tax<<' '<<_imuData.tay<<' '<<_imuData.taz<<std::endl;
                    }
                    imuData = _imuData;
                    return ret;
                } else {
                    return false;
                }
               
            }
            
        }

        if (timeEndPoint < std::time(nullptr) ) {
            // imuPort.closePort();
            return -1;
        }
        
    }
    
}

bool IMU::changeWaitTime(uint32_t delayInS)
{
    _revWaitTime = delayInS;
    return true;
}

bool IMU::isOpen()
{
    return isPortOpen;
}
