#ifndef _PARSEGNSS_
#define _PARSEGNSS_

#include "serial.hpp"
#include <memory>
#include <iostream>
#include <fstream>
#include <mutex>
#include <cstdint>
#include "RingBuffer.hpp"
#include "GNSSMsg.hpp"
#include <map>

#define GNSS_RECV_BUFFER_SIZE 20480

#define UM982_READ_BUFFER_SIZE 10240

#define UM982_FRAME_NMEA_HEAD '$'
#define UM982_FRAME_EXUNI_HEAD '#'
#define UM982_FRAME_TAIL '*'

class GNSS
{
private:
    enum class GNSSdecodeState {
		uninit,
		got_frameHead,
        got_asteriks,
        got_crlf,
		check
	};

    SerialPort gnssPort;
    // std::ofstream logFile;
    std::map<GNSSMSGTYPE, std::ofstream> logFileMap;
    std::map<GNSSMSGTYPE, bool> echoMap;
    bool isNeedLog; // default false
    bool isNeedEcho; // default false
    bool isNMEAFrame;
    bool isPortOpen;
    RingBuffer<uint8_t> _buf;
    uint8_t _gnssFrame[UM982_READ_BUFFER_SIZE];
    uint16_t _gnssDataLen, _gnssFrameLen;
    uint32_t _revWaitTime; // s , default 5 (min) -- 300 (s)
    uint32_t _checkNum;
    GNSSdecodeState _decode_state{GNSSdecodeState::uninit};
    std::mutex _mtx;
    GNSSDataMsg gnssMsg;

    // private method
    bool logMsg(GNSSMSGTYPE msgType);
    bool echoMsg(GNSSMSGTYPE msgType);
    bool CRCcheck();
    bool BCCcheck();
    bool getGNSSDataPtr(const GNSSMSGTYPE &msgType, void*& msgPtr);
    bool parseUM982(); // TODO: factory - product pattern
    GNSSMSGTYPE handledMessage();

public:
    GNSS(const std::string& gnssSerialPath, int baudRate = 115200);
    ~GNSS();
    bool setLogFile(GNSSMSGTYPE msgType, const std::string& logPath);
    bool setEcho(GNSSMSGTYPE msgType, bool flag);
    bool isOpen();
    bool changeWaitTime(uint32_t delayInS);
    int listenInfLoop(); // if not success receive complete gnss frame in revWaitTime return false
    int listen(GNSSMSGTYPE &msgType, void*& msgPtr); // success return 1
    bool getGNSSFreq(GNSSSYSTYPE gnssType, uint32_t freqType, double &freq);
    bool getGNSSData(const GNSSMSGTYPE &msgType, void *msgPtr);
};



#endif