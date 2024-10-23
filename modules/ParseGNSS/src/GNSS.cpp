#include "GNSS.hpp"
#include <cstdlib>
#include <ctime>
#include <cstring>

const uint32_t crc32Table[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

GNSS::GNSS(const std::string& gnssSerialPath, int baudRate) : gnssPort(gnssSerialPath, baudRate), isNeedLog(false), isNeedEcho(false), 
       _revWaitTime(300), _buf(GNSS_RECV_BUFFER_SIZE), isNMEAFrame(false), isPortOpen(false)
{
    if(gnssPort.openPort()) {
        isPortOpen = true;
    }

    logFileMap[GNSSMSGTYPE::NMEA_XXZDA] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXZDA] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXGGA] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXGGA] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXTHS] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXTHS] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXGNS] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXGNS] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXRMC] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXRMC] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXGST] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXGST] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXGSA] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXGSA] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXGSV] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXGSV] = false;
    logFileMap[GNSSMSGTYPE::NMEA_XXVTG] = std::ofstream(); echoMap[GNSSMSGTYPE::NMEA_XXVTG] = false;
    logFileMap[GNSSMSGTYPE::EUNI_KSXT] = std::ofstream(); echoMap[GNSSMSGTYPE::EUNI_KSXT] = false;
    logFileMap[GNSSMSGTYPE::EUNI_OBSVM] = std::ofstream(); echoMap[GNSSMSGTYPE::EUNI_OBSVM] = false;
    logFileMap[GNSSMSGTYPE::EUNI_STATECEF] = std::ofstream(); echoMap[GNSSMSGTYPE::EUNI_STATECEF] = false;
}

GNSS::~GNSS()
{
    // free nmea data
    delete gnssMsg.xNMEAData.pxNMEAZDAData;
    delete gnssMsg.xNMEAData.pxNMEAGGAData;
    delete gnssMsg.xNMEAData.pxNMEATHSData;
    delete gnssMsg.xNMEAData.pxNMEAGNSData;
    delete gnssMsg.xNMEAData.pxNMEARMCData;
    delete gnssMsg.xNMEAData.pxNMEAGSTData;
    delete gnssMsg.xNMEAData.pxNMEAGSVData;
    delete gnssMsg.xNMEAData.pxNMEAVTGData;
    delete gnssMsg.xNMEAData.pxEUNIKSXTData;
    // free extended unicore data
    delete gnssMsg.xEUNIData.pxEUNISATECEFData;
    delete gnssMsg.xEUNIData.pxEUNIOBSVMData;
}

bool GNSS::BCCcheck()
{
    uint8_t usBcc = 0;
    for (size_t i = 1; i < (_gnssDataLen ); i++) {
        usBcc ^= _gnssFrame[i];
    }
    if (usBcc == (_checkNum & 0xFF)) {
        return true;
    } else {
        return false;
    }
    return true;
}

bool GNSS::CRCcheck()
{
    uint16_t iIndex;
    uint32_t ulCRC = 0;

    for (iIndex=1; iIndex < (_gnssDataLen); iIndex++) {
        ulCRC = crc32Table[(ulCRC ^ _gnssFrame[iIndex]) & 0xff] ^ (ulCRC >> 8);
    }

    if (ulCRC == _checkNum) {
        return true;
    } else {
        return false;
    }
}

bool GNSS::parseUM982()
{
    bool ret = false;
    uint8_t curB;
    ret = _buf.isEmpty();
    if (ret == true) {
        return false;
    }

    curB = _buf.front();
    // std::cout << curB;
    switch (_decode_state)
    {
    case GNSSdecodeState::uninit :
        _gnssFrameLen = 0;
        _gnssDataLen = 0;
        if (curB == UM982_FRAME_NMEA_HEAD) {
            isNMEAFrame = true;
            _gnssFrame[_gnssFrameLen++] = curB;
            _decode_state = GNSSdecodeState::got_frameHead;
        } else if(curB == UM982_FRAME_EXUNI_HEAD) {
            isNMEAFrame = false;
            _gnssFrame[_gnssFrameLen++] = curB;
            _decode_state = GNSSdecodeState::got_frameHead;
        } 
        _buf.popOut();
        break;
    case GNSSdecodeState::got_frameHead :
		if (curB == '$' || curB == '#') {
			_decode_state = GNSSdecodeState::got_frameHead;
            _gnssDataLen = 0;

		} else if (curB == '*') {
            _checkNum = 0;
			_decode_state = GNSSdecodeState::got_asteriks;
            // _gnssFrame[_gnssFrameLen++] = curB;
		}

		if (_gnssFrameLen >= (sizeof(_gnssFrame) - 5)) {
			_decode_state = GNSSdecodeState::uninit;
            _gnssDataLen = 0;
			_gnssFrameLen = 0;

		} else {
			_gnssFrame[_gnssFrameLen++] = curB;
            _gnssDataLen++;
		}
        _buf.popOut();
        break;
    case GNSSdecodeState::got_asteriks :
        _gnssFrame[_gnssFrameLen++] = curB;
        if (curB == '\r'||curB == '\n') {
            _decode_state = GNSSdecodeState::got_crlf;
        } else {
            _checkNum = _checkNum << 4;
            if (curB <= '9') {
                _checkNum |= uint32_t(curB - '0');
            } else if (curB <= 'F') {
                _checkNum |= uint32_t(curB - 'A' + 10);
            } else if(curB <= 'f') {
                _checkNum |= uint32_t(curB - 'a' + 10);
            }
            else {
                _decode_state = GNSSdecodeState::uninit;
            }
        }
        _buf.popOut();
        break;
    case GNSSdecodeState::got_crlf :
        _gnssFrame[_gnssFrameLen++] = curB;
        if (curB == '\r'||curB == '\n') {
            
            _decode_state = GNSSdecodeState::check;
            _buf.popOut();
        } else {

            _decode_state = GNSSdecodeState::uninit;
        }
        break;
    case GNSSdecodeState::check :
        _decode_state = GNSSdecodeState::uninit;
        if (isNMEAFrame) { 
            return BCCcheck();
        } else {
            return CRCcheck();
        }
        break;
    }
    return false;
}

GNSSMSGTYPE GNSS::handledMessage()
{
    GNSSMSGTYPE msgType = GNSSMSGTYPE::UNDEFINED;
    char *endp;
	int uiCalcComma = 0;

	for (int i = 0 ; i < _gnssFrameLen; i++) {
		if (_gnssFrame[i] == ',') { uiCalcComma++; }
	}

    if (isNMEAFrame) { // nmea msg and extended unicore with '$'
        char *bufptr = (char *)(_gnssFrame + 6);
        if ((memcmp(_gnssFrame + 1, "KSXT,", 5) == 0) && (uiCalcComma >=18)) { 
            if (gnssMsg.xNMEAData.pxEUNIKSXTData == nullptr) {
                gnssMsg.xNMEAData.pxEUNIKSXTData = new EUNIKSXTData;
            }
            EUNIKSXTData *tmpPtr = gnssMsg.xNMEAData.pxEUNIKSXTData;
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->utc_time = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lon = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lat = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->height = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->pitch = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->track_true = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->vel = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->roll = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->pos_qual = *(bufptr++); 
            }
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_qual = *(bufptr++); 
            }
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->hsolnSVs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));
                bufptr = endp;
            }
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->msolnSVs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->posEast = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->posNorth = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->posUp = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->velEast = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->velNorth = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->velUp = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            msgType = GNSSMSGTYPE::EUNI_KSXT;
            gnssMsg.xNMEAData.isKSXTupdate = true;
        } else if ((memcmp(_gnssFrame + 3, "ZDA,", 4) == 0) && (uiCalcComma == 6)) {
            if (gnssMsg.xNMEAData.pxNMEAZDAData == nullptr) {
                gnssMsg.xNMEAData.pxNMEAZDAData = new NMEAZDAData;
            }
            NMEAZDAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAZDAData;
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->utc_time = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->day = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->month = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->year = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->local_time_off_hour = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->local_time_off_min = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            msgType = GNSSMSGTYPE::NMEA_XXZDA;
            gnssMsg.xNMEAData.isZDAupdate = true;
        } else if ((memcmp(_gnssFrame + 3, "GGA,", 4) == 0) && (uiCalcComma >= 14)) {
            if (gnssMsg.xNMEAData.pxNMEAGGAData == nullptr) {
                gnssMsg.xNMEAData.pxNMEAGGAData = new NMEAGGAData;
            }
            NMEAGGAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGGAData;
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->utc_time = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lat = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ns = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lon = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ew = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->fix_quality = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->num_of_sv = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->hdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->alt = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    while (*(++bufptr) != ',') {} //skip M

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->geoid_h = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    while (*(++bufptr) != ',') {} //skip M

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->dgps_age = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

            msgType = GNSSMSGTYPE::NMEA_XXGGA;
            gnssMsg.xNMEAData.isGGAupdate = true;
        } else if (memcmp(_gnssFrame + 3, "THS,", 4) == 0 && uiCalcComma == 2) {
            if (gnssMsg.xNMEAData.pxNMEATHSData == nullptr) {
                gnssMsg.xNMEAData.pxNMEATHSData = new NMEATHSData;
            }
		    if (bufptr && *(++bufptr) != ',') {
		    	gnssMsg.xNMEAData.pxNMEATHSData->heading_deg = strtof(bufptr, &endp); 
                bufptr = endp;
		    }
            msgType = GNSSMSGTYPE::NMEA_XXTHS;
            gnssMsg.xNMEAData.isTHSupdate = true;
        } else if (memcmp(_gnssFrame + 3, "GNS,", 4) == 0 && uiCalcComma >= 12) {
            if (gnssMsg.xNMEAData.pxNMEAGNSData == nullptr) {
                gnssMsg.xNMEAData.pxNMEAGNSData = new NMEAGNSData;
            }
            NMEAGNSData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGNSData;
            for (size_t i = 0; i < 5; i++)
            {
                tmpPtr->pos_Mode[i] = 'N';
            }
        
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->utc_time = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lat = strtod(bufptr, &endp); 
                bufptr = endp;
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ns = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lon = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ew = *(bufptr++);
            }

            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
		        do {
		        	tmpPtr->pos_Mode[i] = *(bufptr);
		        	i++;
		        } while (*(++bufptr) != ',' && i < 5);
                tmpPtr->pos_Mode[i] = 0;
            }
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->num_of_sv = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->hdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->alt = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

            msgType = GNSSMSGTYPE::NMEA_XXGNS;
            gnssMsg.xNMEAData.isGNSupdate = true;
        } else if (memcmp(_gnssFrame + 3, "RMC,", 4) == 0 && uiCalcComma >= 11) {
            if (gnssMsg.xNMEAData.pxNMEARMCData == nullptr)
            {
                gnssMsg.xNMEAData.pxNMEARMCData = new NMEARMCData;
            }
            NMEARMCData *tmpPtr = gnssMsg.xNMEAData.pxNMEARMCData;
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->utc_time = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Status = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lat = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ns = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lon = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ew = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ground_speed_K = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->track_true = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->nmea_date = static_cast<int>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Mag_var = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            msgType = GNSSMSGTYPE::NMEA_XXRMC;
            gnssMsg.xNMEAData.isRMCupdate = true;
        } else if (memcmp(_gnssFrame + 3, "GST,", 4) == 0 && uiCalcComma == 8) {
            if (gnssMsg.xNMEAData.pxNMEAGSTData == nullptr)
            {
                gnssMsg.xNMEAData.pxNMEAGSTData = new NMEAGSTData;
            }
            NMEAGSTData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSTData;
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->utc_time = strtod(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->rms_err = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->maj_err = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->min_err = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->deg_from_north = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lat_err = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->lon_err = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->alt_err = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            msgType = GNSSMSGTYPE::NMEA_XXGST;
            gnssMsg.xNMEAData.isGSTupdate = true;
        } else if ((memcmp(_gnssFrame + 3, "GSA,", 4) == 0) && (uiCalcComma >= 17)) {
            if (gnssMsg.xNMEAData.pxNMEAGSAData == nullptr) {
                gnssMsg.xNMEAData.pxNMEAGSAData = new NMEAGSAData;
            }
            NMEAGSAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSAData;
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->M_pos = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->fix_mode = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    for (int y = 0; y < 12; y++) {
		    	if (bufptr && *(++bufptr) != ',') {
                    tmpPtr->sat_id[y] = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                    }
		    }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->pdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->hdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->vdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            msgType = GNSSMSGTYPE::NMEA_XXGSA;
            gnssMsg.xNMEAData.isGSAupdate = true;
        } else if ((memcmp(_gnssFrame + 3, "GSV,", 4) == 0)) {
            if (gnssMsg.xNMEAData.pxNMEAGSVData == nullptr) {
                gnssMsg.xNMEAData.pxNMEAGSVData = new NMEAGSVData;
            }
            NMEAGSVData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSVData;
		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->all_page_num = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->this_page_num = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->tot_sv_visible = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }

            if ((tmpPtr->this_page_num < 1) || (tmpPtr->this_page_num > tmpPtr->all_page_num)) {
                msgType = GNSSMSGTYPE::UNDEFINED;
                gnssMsg.xNMEAData.isGSVupdate = false;
                // break;
                return msgType;
            }
		    if (memcmp(_gnssFrame, "$GP", 3) == 0) {
		    	tmpPtr->gnsstype = GNSSSYSTYPE::GPS;
		    } else if (memcmp(_gnssFrame, "$GL", 3) == 0) {
		    	tmpPtr->gnsstype = GNSSSYSTYPE::GLONASS;
		    } else if (memcmp(_gnssFrame, "$GA", 3) == 0) {
		    	tmpPtr->gnsstype = GNSSSYSTYPE::GALILEIO;
		    } else if (memcmp(_gnssFrame, "$GB", 3) == 0) {
		    	tmpPtr->gnsstype = GNSSSYSTYPE::BDS;
		    } else if (memcmp(_gnssFrame, "$GQ", 3) == 0) {
		    	tmpPtr->gnsstype = GNSSSYSTYPE::QZSS;
		    } else if (memcmp(_gnssFrame, "$GI", 3) == 0) {
		    	tmpPtr->gnsstype = GNSSSYSTYPE::NAVIC;
		    } else {
                tmpPtr->gnsstype = GNSSSYSTYPE::UNDEFINED;
            }
            size_t end = 4;
            if (tmpPtr->this_page_num == tmpPtr->all_page_num) {
                end =  tmpPtr->tot_sv_visible - (tmpPtr->this_page_num - 1) * 4;
            }
            for (size_t i = 0; i < end; i++) {
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->xSat[i].svid = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }

				if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->xSat[i].elevation = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }

				if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->xSat[i].azimuth = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }

				if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->xSat[i].snr = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->signalID = *(bufptr++); 
            }

            msgType = GNSSMSGTYPE::NMEA_XXGSV;
            gnssMsg.xNMEAData.isGSVupdate = true;
        } else if ((memcmp(_gnssFrame + 3, "VTG,", 4) == 0) && (uiCalcComma >= 8)) {
            if (gnssMsg.xNMEAData.pxNMEAVTGData == nullptr)
            {
                gnssMsg.xNMEAData.pxNMEAVTGData = new NMEAVTGData;
            }
            NMEAVTGData *tmpPtr = gnssMsg.xNMEAData.pxNMEAVTGData;
		    if (bufptr && *(++bufptr) != ',') {
                tmpPtr->track_true = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->T = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') {
                tmpPtr->track_mag = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->M = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ground_speed = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->N = *(bufptr++); 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->ground_speed_K = strtof(bufptr, &endp); 
                bufptr = endp; 
            }

		    if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->K = *(bufptr++); 
            }
            msgType = GNSSMSGTYPE::NMEA_XXVTG;
            gnssMsg.xNMEAData.isVTGupdate = true;
        }
    } else { // extended unicore msg with '#'
        if ((memcmp(_gnssFrame + 1, "OBSVMA,", 7) == 0) && (uiCalcComma >=10)) {
            char *bufptr = (char *)(_gnssFrame + 7);
            if (gnssMsg.xEUNIData.pxEUNIOBSVMData == nullptr)
            {
                gnssMsg.xEUNIData.pxEUNIOBSVMData = new EUNIOBSVMData;
            }
            EUNIOBSVMData *tmpPtr = gnssMsg.xEUNIData.pxEUNIOBSVMData;
#ifdef USE_UM982HEADER
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.cpuDle = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeSys[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 4);
                tmpPtr->xHeader.timeSys[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeStatus[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 8);
                tmpPtr->xHeader.timeStatus[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.wn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.ms = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            while (*(++bufptr) != ','); // skip reserved
            while (*(++bufptr) != ','); // skip version reserved
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.leapSec = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.outDly = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
#endif
            for (; *bufptr != ';'; bufptr++); // skip header
            if (*bufptr == ';' && *(++bufptr) != ';') { 
                tmpPtr->obsNum = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (tmpPtr->obsNum == 0) {
                msgType = GNSSMSGTYPE::UNDEFINED;
                gnssMsg.xEUNIData.isOBSVMupdate = false;
                // break;
                return msgType;
            }
            
            uint32_t loop1, loop2 = 0;
            uint32_t curSize = tmpPtr->obsSVInfo.size();
            if (tmpPtr->obsNum > curSize) {
                loop1 = curSize;
                loop2 = tmpPtr->obsNum - curSize;
            } else {
                loop1 = tmpPtr->obsNum;
                loop2 = 0;
            }
            for (size_t i = 0; i < loop1; i++) {
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].sysFreq = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].prn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].psr = strtod(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].adr = strtod(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].psr_std = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].adr_std = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].dopp = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].CN0 = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                while (*(++bufptr) != ','); // skip reserved
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].locktime = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->obsSVInfo[i].trStatus = static_cast<uint32_t>(strtol(bufptr, &endp, 16)); 
                    bufptr = endp; 
                }
            }
            for (size_t i = 0; i < loop2; i++) {
                EUNISATOBSV tmpObsInfo;
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.sysFreq = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.prn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.psr = strtod(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.adr = strtod(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.psr_std = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.adr_std = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.dopp = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.CN0 = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                while (*(++bufptr) != ','); // skip reserved
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.locktime = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpObsInfo.trStatus = static_cast<uint32_t>(strtol(bufptr, &endp, 16)); 
                    bufptr = endp; 
                }
                tmpPtr->obsSVInfo.push_back(tmpObsInfo);
            }
            msgType = GNSSMSGTYPE::EUNI_OBSVM;
            gnssMsg.xEUNIData.isOBSVMupdate = true;
        } else if ((memcmp(_gnssFrame + 1, "SATECEFA,", 9) == 0) && (uiCalcComma >= 10)) {
            char *bufptr = (char *)(_gnssFrame + 10);
            if (gnssMsg.xEUNIData.pxEUNISATECEFData == nullptr) {
                gnssMsg.xEUNIData.pxEUNISATECEFData = new EUNISATECEFData;
            }
            EUNISATECEFData *tmpPtr = gnssMsg.xEUNIData.pxEUNISATECEFData;
#ifdef USE_UM982HEADER
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.cpuDle = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeSys[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 4);
                tmpPtr->xHeader.timeSys[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeStatus[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 8);
                tmpPtr->xHeader.timeStatus[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.wn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.ms = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            while (*(++bufptr) != ','); // skip reserved
            while (*(++bufptr) != ','); // skip version reserved
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.leapSec = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.outDly = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
#endif
            for (; *bufptr != ';'; bufptr++); // skip header
            if (*bufptr == ';' && *(++bufptr) != ';') { 
                tmpPtr->satNum = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (tmpPtr->satNum == 0) {
                msgType = GNSSMSGTYPE::UNDEFINED;
                gnssMsg.xEUNIData.isOBSVMupdate = false;
                // break;
                return msgType;
            }
            uint32_t loop1, loop2 = 0;
            uint32_t curSize = tmpPtr->satECEFInfo.size();
            if (tmpPtr->satNum > curSize) {
                loop1 = curSize;
                loop2 = tmpPtr->satNum - curSize;
            } else {
                loop1 = tmpPtr->satNum;
                loop2 = 0;
            }
            for (size_t i = 0; i < loop1; i++) {
                if (bufptr && *(++bufptr) != ',') { 
                    int j = 0;
                    uint32_t sysChaSum = 0;
                    do
                    {
                        tmpPtr->satECEFInfo[i].GNSS_SYS[j] = *(bufptr);
                        sysChaSum = sysChaSum + *(bufptr);
                        j++;
                    } while (*(++bufptr) != ',' && j < 15);
                    tmpPtr->satECEFInfo[i].GNSS_SYS[j] = 0;
                    switch (sysChaSum)
                    {
                    case EUNI_STATECEF_GPSCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::GPS;
                        break;
                    case EUNI_STATECEF_GLONASSCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::GLONASS;
                        break;
                    case EUNI_STATECEF_SBASCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::SBAS;
                        break;
                    case EUNI_STATECEF_BDSCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::BDS;
                        break;
                    case EUNI_STATECEF_GALILEOCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::GALILEIO;
                        break;
                    case EUNI_STATECEF_QZSSCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::QZSS;
                        break;
                    case EUNI_STATECEF_NAVICCHARSUM:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::NAVIC;
                        break;
                    default:
                        tmpPtr->satECEFInfo[i].sysType = (uint32_t)GNSSSYSTYPE::UNDEFINED;
                        break;
                    }
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].prn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].satCoordX = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].satCoordY = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].satCoordZ = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].satClk = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].lonoDly = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->satECEFInfo[i].tropDly = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                while (*(++bufptr) != ','); // skip reserved
                
            }
            for (size_t i = 0; i < loop2; i++) {
                EUNIECEFSAT satInfo;
                if (bufptr && *(++bufptr) != ',') { 
                    int j = 0;
                    uint32_t sysChaSum = 0;
                    do
                    {
                        satInfo.GNSS_SYS[j] = *(bufptr);
                        sysChaSum = sysChaSum + *(bufptr);
                        j++;
                    } while (*(++bufptr) != ',' && j < 15);
                    satInfo.GNSS_SYS[j] = 0;
                    switch (sysChaSum)
                    {
                    case EUNI_STATECEF_GPSCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::GPS;
                        break;
                    case EUNI_STATECEF_GLONASSCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::GLONASS;
                        break;
                    case EUNI_STATECEF_SBASCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::SBAS;
                        break;
                    case EUNI_STATECEF_BDSCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::BDS;
                        break;
                    case EUNI_STATECEF_GALILEOCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::GALILEIO;
                        break;
                    case EUNI_STATECEF_QZSSCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::QZSS;
                        break;
                    case EUNI_STATECEF_NAVICCHARSUM:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::NAVIC;
                        break;
                    default:
                        satInfo.sysType = (uint32_t)GNSSSYSTYPE::UNDEFINED;
                        break;
                    }
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.prn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.satCoordX = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.satCoordY = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.satCoordZ = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.satClk = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.lonoDly = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                if (bufptr && *(++bufptr) != ',') { 
                    satInfo.tropDly = strtof(bufptr, &endp);
                    bufptr = endp; 
                }
                while (*(++bufptr) != ','); // skip reserved
                tmpPtr->satECEFInfo.push_back(satInfo);
            }
            msgType = GNSSMSGTYPE::EUNI_STATECEF;
            gnssMsg.xEUNIData.isSATEECEFupdate = true;
        } else if ((memcmp(_gnssFrame + 1, "STADOPA,", 8) == 0) && (uiCalcComma >= 10)) {
            char *bufptr = (char *)(_gnssFrame + 9);
            if (gnssMsg.xEUNIData.pxEUNISATDOPData == nullptr)
            {
                gnssMsg.xEUNIData.pxEUNISATDOPData = new EUNISATDOPData;
            }
            EUNISATDOPData *tmpPtr = gnssMsg.xEUNIData.pxEUNISATDOPData;
#ifdef USE_UM982HEADER
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.cpuDle = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeSys[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 4);
                tmpPtr->xHeader.timeSys[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeStatus[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 8);
                tmpPtr->xHeader.timeStatus[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.wn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.ms = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            while (*(++bufptr) != ','); // skip reserved
            while (*(++bufptr) != ','); // skip version reserved
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.leapSec = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.outDly = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
#endif
            for (; *bufptr != ';'; bufptr++); // skip header
            if (*bufptr == ';' && *(++bufptr) != ';') { 
                tmpPtr->Itow = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->gdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Pdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Tdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Vdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Hdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Ndop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->Edop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->cutoff = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->PRNnums = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
            }
            uint32_t loop1, loop2 = 0;
            uint32_t curSize = tmpPtr->prns.size();
            if (tmpPtr->PRNnums > curSize) {
                loop1 = curSize;
                loop2 = tmpPtr->PRNnums - curSize;
            } else {
                loop1 = tmpPtr->PRNnums;
                loop2 = 0;
            }
            
            for (size_t i = 0; i < loop1; i++)
            {
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->prns[i] = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
            }
            for (size_t i = 0; i < loop2; i++)
            {
                if (bufptr && *(++bufptr) != ',') { 
                    uint16_t tmpPrn = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                    tmpPtr->prns.push_back(tmpPrn);
                    bufptr = endp; 
                }
            }
            msgType = GNSSMSGTYPE::EUNI_STADOP;
            gnssMsg.xEUNIData.isSATDOPupdate = true;
        } else if ((memcmp(_gnssFrame + 1, "PVTSLNA,", 8) == 0) && (uiCalcComma >= 10)) {
            char *bufptr = (char *)(_gnssFrame + 9);
            if (gnssMsg.xEUNIData.pxEUNIPVTSLNData == nullptr)
            {
                gnssMsg.xEUNIData.pxEUNIPVTSLNData = new EUNIPVTSLNData;
            }
            EUNIPVTSLNData *tmpPtr = gnssMsg.xEUNIData.pxEUNIPVTSLNData;
#ifdef USE_UM982HEADER
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.cpuDle = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeSys[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 4);
                tmpPtr->xHeader.timeSys[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                int i = 0;
                do
                {
                    tmpPtr->xHeader.timeStatus[i] = *(bufptr);
                    i++;
                } while (*(++bufptr) != ',' && i < 8);
                tmpPtr->xHeader.timeStatus[i] = 0;
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.wn = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.ms = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            while (*(++bufptr) != ','); // skip reserved
            while (*(++bufptr) != ','); // skip version reserved
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.leapSec = static_cast<uint8_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->xHeader.outDly = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
#endif
            for (; *bufptr != ';'; bufptr++); // skip header
            if (*bufptr == ';' && *(++bufptr) != ';') { 
                int j = 0;
                uint32_t sysChaSum = 0;
                do
                {
                    sysChaSum = sysChaSum + *(bufptr);
                    j++;
                } while (*(++bufptr) != ',' && j < 15);
                switch (sysChaSum)
                {
                case EUNI_PVTSLN_NONESUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::NONE;
                    break;
                case EUNI_PVTSLN_FIXEDPOSSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::FIXEDPOS;
                    break;
                case EUNI_PVTSLN_FIXEDHEIGHTSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::FIXEDHEIGHT;
                    break;
                case EUNI_PVTSLN_DOPPLER_VELOCITYSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::DOPPLER_VELOCITY;
                    break;
                case EUNI_PVTSLN_SINGLESUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::SINGLE;
                    break;
                case EUNI_PVTSLN_PSRDIFFSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::PSRDIFF;
                    break;
                case EUNI_PVTSLN_SBASSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::SBAS;
                    break;
                case EUNI_PVTSLN_L1_FLOATUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::L1_FLOAT;
                    break;
                case EUNI_PVTSLN_IONOFREE_FLOATSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::IONOFREE_FLOAT;
                    break;
                case EUNI_PVTSLN_NARROW_FLOATSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::NARROW_FLOAT;
                    break;
                case EUNI_PVTSLN_L1_INTSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::L1_INT;
                    break;
                case EUNI_PVTSLN_WIDE_INTSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::WIDE_INT;
                    break;
                case EUNI_PVTSLN_NARROW_INTSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::NARROW_INT;
                    break;
                case EUNI_PVTSLN_INSSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::INS;
                    break;
                case EUNI_PVTSLN_INS_PSRSPSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::INS_PSRSP;
                    break;
                case EUNI_PVTSLN_INS_PSRDIFFSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::INS_PSRDIFF;
                    break;
                case EUNI_PVTSLN_INS_RTKFLOATSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::INS_RTKFLOAT;
                    break;
                case EUNI_PVTSLN_INS_RTKFIXEDSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::INS_RTKFIXED;
                    break;
                case EUNI_PVTSLN_PPP_CONVERGINGSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::PPP_CONVERGING;
                    break;
                case EUNI_PVTSLN_PPPSUM:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::PPP;
                    break;
                default:
                    tmpPtr->bestpos_type = (uint32_t)GNSSLOCTYPET04::UNDEFINED;
                    break;
                }
            }
            
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_hgt = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_lat = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_lon = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_hgtstd = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_latstd = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_lonstd = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_diffage = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                int j = 0;
                uint32_t sysChaSum = 0;
                do
                {
                    sysChaSum = sysChaSum + *(bufptr);
                    j++;
                } while (*(++bufptr) != ',' && j < 15);
                switch (sysChaSum)
                {
                case EUNI_PVTSLN_NONESUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::NONE;
                    break;
                case EUNI_PVTSLN_FIXEDPOSSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::FIXEDPOS;
                    break;
                case EUNI_PVTSLN_FIXEDHEIGHTSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::FIXEDHEIGHT;
                    break;
                case EUNI_PVTSLN_DOPPLER_VELOCITYSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::DOPPLER_VELOCITY;
                    break;
                case EUNI_PVTSLN_SINGLESUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::SINGLE;
                    break;
                case EUNI_PVTSLN_PSRDIFFSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::PSRDIFF;
                    break;
                case EUNI_PVTSLN_SBASSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::SBAS;
                    break;
                case EUNI_PVTSLN_L1_FLOATUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::L1_FLOAT;
                    break;
                case EUNI_PVTSLN_IONOFREE_FLOATSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::IONOFREE_FLOAT;
                    break;
                case EUNI_PVTSLN_NARROW_FLOATSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::NARROW_FLOAT;
                    break;
                case EUNI_PVTSLN_L1_INTSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::L1_INT;
                    break;
                case EUNI_PVTSLN_WIDE_INTSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::WIDE_INT;
                    break;
                case EUNI_PVTSLN_NARROW_INTSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::NARROW_INT;
                    break;
                case EUNI_PVTSLN_INSSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::INS;
                    break;
                case EUNI_PVTSLN_INS_PSRSPSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::INS_PSRSP;
                    break;
                case EUNI_PVTSLN_INS_PSRDIFFSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::INS_PSRDIFF;
                    break;
                case EUNI_PVTSLN_INS_RTKFLOATSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::INS_RTKFLOAT;
                    break;
                case EUNI_PVTSLN_INS_RTKFIXEDSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::INS_RTKFIXED;
                    break;
                case EUNI_PVTSLN_PPP_CONVERGINGSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::PPP_CONVERGING;
                    break;
                case EUNI_PVTSLN_PPPSUM:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::PPP;
                    break;
                default:
                    tmpPtr->psrpos_type = (uint32_t)GNSSLOCTYPET04::UNDEFINED;
                    break;
                }
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrpos_hgt = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrpos_lat = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrpos_lon = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->undulation = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_svs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->bestpos_solnsvs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrpos_svs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrpos_solnsvs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp;  
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrvel_north = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrvel_east = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->psrvel_ground = strtod(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                int j = 0;
                uint32_t sysChaSum = 0;
                do
                {
                    sysChaSum = sysChaSum + *(bufptr);
                    j++;
                } while (*(++bufptr) != ',' && j < 15);
                switch (sysChaSum)
                {
                case EUNI_PVTSLN_NONESUM:
                    tmpPtr->heading_type = (uint32_t)GNSSHEADTYPET05::NONE;
                    break;
                case EUNI_PVTSLN_SOL_COMPUTEDSUM :
                    tmpPtr->heading_type = (uint32_t)GNSSHEADTYPET05::SOL_COMPUTED;
                    break;
                case EUNI_PVTSLN_INSUFFICIENT_OBSSUM :
                    tmpPtr->heading_type = (uint32_t)GNSSHEADTYPET05::INSUFFICIENT_OBS;
                    break;
                case EUNI_PVTSLN_NO_CONVERGENCESUM :
                    tmpPtr->heading_type = (uint32_t)GNSSHEADTYPET05::NO_CONVERGENCE;
                    break;
                case EUNI_PVTSLN_COV_TRACESUM:
                    tmpPtr->heading_type = (uint32_t)GNSSHEADTYPET05::COV_TRACE;
                    break;
                default:
                    tmpPtr->heading_type = (uint32_t)GNSSHEADTYPET05::UNDEFINED;
                    break;
                }
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_length = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_degree = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_pitch = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_trackedsvs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_solnsvs = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_ggl1 = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->heading_ggl1l2 = static_cast<uint8_t>(strtol(bufptr, &endp, 10));  
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->gdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->pdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->hdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->htdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->tdop = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->cutoff = strtof(bufptr, &endp); 
                bufptr = endp; 
            }
            if (bufptr && *(++bufptr) != ',') { 
                tmpPtr->PRNnums = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                bufptr = endp; 
            }
            uint32_t loop1, loop2 = 0;
            uint32_t curSize = tmpPtr->prns.size();
            if (tmpPtr->PRNnums > curSize) {
                loop1 = curSize;
                loop2 = tmpPtr->PRNnums - curSize;
            } else {
                loop1 = tmpPtr->PRNnums;
                loop2 = 0;
            }
            for (size_t i = 0; i < loop1; i++)
            {
                if (bufptr && *(++bufptr) != ',') { 
                    tmpPtr->prns[i] = static_cast<uint16_t>(strtol(bufptr, &endp, 10)); 
                    bufptr = endp; 
                }
            }
            for (size_t i = 0; i < loop2; i++)
            {
                if (bufptr && *(++bufptr) != ',') { 
                    uint16_t tmpPrn = static_cast<uint32_t>(strtol(bufptr, &endp, 10)); 
                    tmpPtr->prns.push_back(tmpPrn);
                    bufptr = endp; 
                }
            }
            msgType = GNSSMSGTYPE::EUNI_PVTSLN;
            gnssMsg.xEUNIData.isPVTSLNupdate = true;
        }
    }

    return msgType;
}

bool GNSS::setLogFile(GNSSMSGTYPE msgType, const std::string& logPath)
{
    bool ret = false;
    if (logPath.empty()) {
        // logFile.close();
        logFileMap[msgType].close();
        isNeedLog = false;
        return true;
    }
    // logFile.close();
    logFileMap[msgType].close();
    logFileMap[msgType].open(logPath);
    if (logFileMap[msgType].is_open()) {
        isNeedLog = true;
        // logFile << "utc lon lat heigh velE velN velU pseCode Carrier ddoper"<< std::endl;
       switch (msgType)
        {
        case GNSSMSGTYPE::NMEA_XXZDA :
            logFileMap[msgType] << "utc_time day month year local_time_off_hour local_time_off_min" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGGA :
            logFileMap[msgType] << "utc_time lat ns lon ew fix_quality num_of_sv hdop alt geoid_h dgps_age" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXTHS :
            logFileMap[msgType] << "local_time heading_deg" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGNS :
            logFileMap[msgType] << "utc_time lat ns lon ew pos_Mode num_of_sv hdop alt" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXRMC :
            logFileMap[msgType] << "utc_time Status lat ns lon ew ground_speed_K track_true nmea_date Mag_var" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGST :
            logFileMap[msgType] << "utc_time rms_err maj_err ns min_err deg_from_north lat_err lon_err alt_err" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGSA :
            logFileMap[msgType] << "local_time M_pos fix_mode sat_id pdop hdop vdop" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGSV :
            logFileMap[msgType] << "local_time gnsstype tot_sv_visible svid elevation azimuth snr signalID" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXVTG :
            logFileMap[msgType] << "local_time track_true T track_mag M ground_speed N ground_speed_K K" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_KSXT :
            logFileMap[msgType] << "utc_time lon lat height track_true vel pos_qual heading_qual hsolnSVs msolnSVs" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_OBSVM :
            logFileMap[msgType] << "local_time obsNum sysFreq prn psr adr psr_std adr_std dopp CN0 locktime trStatus" << std::endl;
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_STATECEF :
            logFileMap[msgType] << "local_time satNum GNSS_SYS prn satCoordX satCoordY satCoordZ satClk lonoDly tropDly" << std::endl;
            ret = true;
            break;
        default:
            logFileMap[msgType].close();
            ret = false;
            break;
        }
    } else {
        return false;
    }
    return ret;
}

bool GNSS::setEcho(GNSSMSGTYPE msgType, bool flag)
{
    isNeedEcho = flag;
    echoMap[msgType] = flag;
    return true;
}

bool GNSS::logMsg(GNSSMSGTYPE msgType)
{
    bool ret = false;
    switch (msgType)
    {
    case GNSSMSGTYPE::NMEA_XXZDA :
        if (logFileMap[msgType].is_open()) {
            NMEAZDAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAZDAData;
            logFileMap[msgType] << tmpPtr->utc_time << ' ' << unsigned(tmpPtr->day) << ' '
                << unsigned(tmpPtr->month) << ' ' << tmpPtr->year << ' ' << unsigned(tmpPtr->local_time_off_hour)
                << ' ' << unsigned(tmpPtr->local_time_off_min) << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGGA :
        if (logFileMap[msgType].is_open()) {
            NMEAGGAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGGAData;
            logFileMap[msgType] << tmpPtr->utc_time << ' ' << tmpPtr->lat << ' ' << tmpPtr->ns
                << tmpPtr->lon << ' ' << tmpPtr->ew << ' ' << unsigned(tmpPtr->fix_quality) << ' ' 
                << unsigned(tmpPtr->num_of_sv) << ' ' << tmpPtr->hdop << ' ' << tmpPtr->geoid_h << ' '
                << tmpPtr->dgps_age << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXTHS :
        if (logFileMap[msgType].is_open()) {
            NMEATHSData *tmpPtr = gnssMsg.xNMEAData.pxNMEATHSData;
            logFileMap[msgType] << std::time(nullptr) << ' ' << tmpPtr->heading_deg << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGNS :
        if (logFileMap[msgType].is_open()) {
            NMEAGNSData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGNSData;
            logFileMap[msgType] << tmpPtr->utc_time << ' ' << tmpPtr->lat << ' ' << tmpPtr->ns << ' '
                << tmpPtr->lon << ' ' << tmpPtr->ew << ' ' << tmpPtr->pos_Mode << ' ' << unsigned(tmpPtr-> num_of_sv)
                << ' ' << tmpPtr->hdop << ' ' << tmpPtr->alt << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXRMC :
        if (logFileMap[msgType].is_open()) {
            NMEARMCData *tmpPtr = gnssMsg.xNMEAData.pxNMEARMCData;
            logFileMap[msgType] << tmpPtr->utc_time << ' ' << tmpPtr->Status << ' ' << tmpPtr->lat << ' '
                << tmpPtr->ns << ' ' << tmpPtr->lon << ' ' << tmpPtr->ew << ' ' << tmpPtr-> ground_speed_K
                << ' ' << tmpPtr->track_true << ' ' << tmpPtr->nmea_date << ' ' << tmpPtr->Mag_var << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGST :
        if (logFileMap[msgType].is_open()) {
            NMEAGSTData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSTData;
            logFileMap[msgType] << tmpPtr->utc_time << ' ' << tmpPtr->rms_err << ' ' << tmpPtr->maj_err << ' '
                << tmpPtr->min_err << ' ' << tmpPtr->deg_from_north << ' ' << tmpPtr-> lat_err
                << ' ' << tmpPtr->lon_err << ' ' << tmpPtr->alt_err << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGSA :
        if (logFileMap[msgType].is_open()) {
            NMEAGSAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSAData;
            logFileMap[msgType] << std::time(nullptr) << ' ' << tmpPtr->M_pos << ' ' << unsigned(tmpPtr->fix_mode) << ' '
                << tmpPtr->sat_id << ' ' << tmpPtr->pdop << ' ' << tmpPtr->hdop << ' ' << tmpPtr-> vdop << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGSV :
        if (logFileMap[msgType].is_open()) {
            NMEAGSVData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSVData;
            size_t end = 4;
            if (tmpPtr->this_page_num == tmpPtr->all_page_num) {
                end =  tmpPtr->tot_sv_visible - (tmpPtr->this_page_num - 1) * 4;
            }
            for (size_t i = 0; i < end ; i++) {
                logFileMap[msgType] << std::time(nullptr) << ' ' << unsigned(tmpPtr->gnsstype) << ' '
                    << unsigned(tmpPtr->tot_sv_visible) << ' ' << unsigned(tmpPtr->xSat[i].svid) << ' '
                    << tmpPtr->xSat[i].elevation << ' ' << tmpPtr->xSat[i].azimuth << ' '
                    << unsigned(tmpPtr->xSat[i].snr) << ' ' << unsigned(tmpPtr->signalID) << std::endl;
            }
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXVTG :
        if (logFileMap[msgType].is_open()) {
            NMEAVTGData *tmpPtr = gnssMsg.xNMEAData.pxNMEAVTGData;
            logFileMap[msgType] << std::time(nullptr) << ' ' << tmpPtr->track_true << ' ' << tmpPtr->T << ' '
                << tmpPtr->track_mag << ' ' << tmpPtr->M << ' ' << tmpPtr->ground_speed << ' ' << tmpPtr-> N 
                << ' ' << tmpPtr->ground_speed_K << ' ' << tmpPtr->K << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_KSXT :
        if (logFileMap[msgType].is_open()) {
            EUNIKSXTData *tmpPtr = gnssMsg.xNMEAData.pxEUNIKSXTData;
            logFileMap[msgType] << tmpPtr->utc_time << ' ' << tmpPtr->lon << ' ' << tmpPtr->lat << ' '
                << tmpPtr->height << ' ' << tmpPtr->track_true << ' ' << tmpPtr->vel << ' ' 
                << unsigned(tmpPtr-> pos_qual) << ' ' << unsigned(tmpPtr->heading_qual) 
                << ' ' << unsigned(tmpPtr->hsolnSVs) << ' ' << unsigned(tmpPtr->msolnSVs) << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_OBSVM :
        if (logFileMap[msgType].is_open()) {
            EUNIOBSVMData *tmpPtr = gnssMsg.xEUNIData.pxEUNIOBSVMData;
            for (size_t i = 0; i < (tmpPtr->obsNum) ; i++) {
                logFileMap[msgType] << std::time(nullptr) << ' ' << tmpPtr->obsNum << ' '
                    << unsigned(tmpPtr->obsSVInfo[i].sysFreq) << ' ' << tmpPtr->obsSVInfo[i].prn << ' '
                    << tmpPtr->obsSVInfo[i].psr << ' ' << tmpPtr->obsSVInfo[i].adr << ' '
                    << unsigned(tmpPtr->obsSVInfo[i].psr_std) << ' ' << unsigned(tmpPtr->obsSVInfo[i].adr_std) << ' '
                    << tmpPtr->obsSVInfo[i].dopp << ' ' << unsigned(tmpPtr->obsSVInfo[i].CN0) << ' '
                    << tmpPtr->obsSVInfo[i].locktime << ' ' << tmpPtr->obsSVInfo[i].trStatus << std::endl;
            }
        }
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_STATECEF :
        if (logFileMap[msgType].is_open()) {
            EUNISATECEFData *tmpPtr = gnssMsg.xEUNIData.pxEUNISATECEFData;
            for (size_t i = 0; i < (tmpPtr->satNum) ; i++) {
                logFileMap[msgType] << std::time(nullptr) << ' ' << tmpPtr->satNum << ' '
                    << tmpPtr->satECEFInfo[i].GNSS_SYS << ' ' << tmpPtr->satECEFInfo[i].prn << ' '
                    << tmpPtr->satECEFInfo[i].satCoordX << ' ' << tmpPtr->satECEFInfo[i].satCoordY << ' '
                    << tmpPtr->satECEFInfo[i].satCoordZ << ' ' << tmpPtr->satECEFInfo[i].satClk << ' '
                    << tmpPtr->satECEFInfo[i].lonoDly << ' ' << tmpPtr->satECEFInfo[i].tropDly << std::endl;
            }
        }
        ret = true;
        break;
    default:
        ret = false;
        break;
    }
    return ret;
}

bool GNSS::echoMsg(GNSSMSGTYPE msgType)
{
    bool ret = false;
    switch (msgType)
    {
    case GNSSMSGTYPE::NMEA_XXZDA :
        if (echoMap[msgType]) {
            NMEAZDAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAZDAData;
            std::cout << tmpPtr->utc_time << ' ' << (int)tmpPtr->day << ' '
                << (int)tmpPtr->month << ' ' << tmpPtr->year << ' ' << (int)tmpPtr->local_time_off_hour
                << ' ' << (int)tmpPtr->local_time_off_min << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGGA :
        if (echoMap[msgType]) {
            NMEAGGAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGGAData;
            std::cout << tmpPtr->utc_time << ' ' << tmpPtr->lat << ' ' << tmpPtr->ns
                << tmpPtr->lon << ' ' << tmpPtr->ew << ' ' << (int)tmpPtr->fix_quality << ' ' 
                << (int)tmpPtr->num_of_sv << ' ' << tmpPtr->hdop << ' ' << tmpPtr->geoid_h << ' '
                << tmpPtr->dgps_age << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXTHS :
        if (echoMap[msgType]) {
            NMEATHSData *tmpPtr = gnssMsg.xNMEAData.pxNMEATHSData;
            std::cout << std::time(nullptr) << ' ' << tmpPtr->heading_deg << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGNS :
        if (echoMap[msgType]) {
            NMEAGNSData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGNSData;
            std::cout << tmpPtr->utc_time << ' ' << tmpPtr->lat << ' ' << tmpPtr->ns << ' '
                << tmpPtr->lon << ' ' << tmpPtr->ew << ' ' << tmpPtr->pos_Mode << ' ' << (int)tmpPtr-> num_of_sv
                << ' ' << tmpPtr->hdop << ' ' << tmpPtr->alt << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXRMC :
        if (echoMap[msgType]) {
            NMEARMCData *tmpPtr = gnssMsg.xNMEAData.pxNMEARMCData;
            std::cout << tmpPtr->utc_time << ' ' << tmpPtr->Status << ' ' << tmpPtr->lat << ' '
                << tmpPtr->ns << ' ' << tmpPtr->lon << ' ' << tmpPtr->ew << ' ' << tmpPtr-> ground_speed_K
                << ' ' << tmpPtr->track_true << ' ' << tmpPtr->nmea_date << ' ' << tmpPtr->Mag_var << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGST :
        if (echoMap[msgType]) {
            NMEAGSTData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSTData;
            std::cout << tmpPtr->utc_time << ' ' << tmpPtr->rms_err << ' ' << tmpPtr->maj_err << ' '
                << tmpPtr->min_err << ' ' << tmpPtr->deg_from_north << ' ' << tmpPtr-> lat_err
                << ' ' << tmpPtr->lon_err << ' ' << tmpPtr->alt_err << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGSA :
        if (echoMap[msgType]) {
            NMEAGSAData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSAData;
            std::cout << std::time(nullptr) << ' ' << (int)tmpPtr->M_pos << ' ' << tmpPtr->fix_mode << ' '
                << tmpPtr->sat_id << ' ' << tmpPtr->pdop << ' ' << tmpPtr->hdop << ' ' << tmpPtr-> vdop << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGSV :
        if (echoMap[msgType]) {
            NMEAGSVData *tmpPtr = gnssMsg.xNMEAData.pxNMEAGSVData;
            size_t end = 4;
            if (tmpPtr->this_page_num == tmpPtr->all_page_num) {
                end =  tmpPtr->tot_sv_visible - (tmpPtr->this_page_num - 1) * 4;
            }
            for (size_t i = 0; i < end ; i++) {
                std::cout << std::time(nullptr) << ' ' << unsigned(tmpPtr->gnsstype) << ' '
                    << unsigned(tmpPtr->tot_sv_visible) << ' ' << unsigned(tmpPtr->xSat[i].svid) << ' '
                    << unsigned(tmpPtr->xSat[i].elevation) << ' ' << tmpPtr->xSat[i].azimuth << ' '
                    << unsigned(tmpPtr->xSat[i].snr) << ' ' << unsigned(tmpPtr->signalID) << std::endl;
            }
        }
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXVTG :
        if (echoMap[msgType]) {
            NMEAVTGData *tmpPtr = gnssMsg.xNMEAData.pxNMEAVTGData;
            std::cout << std::time(nullptr) << ' ' << tmpPtr->track_true << ' ' << tmpPtr->T << ' '
                << tmpPtr->track_mag << ' ' << tmpPtr->M << ' ' << tmpPtr->ground_speed << ' ' << tmpPtr-> N 
                << ' ' << tmpPtr->ground_speed_K << ' ' << tmpPtr->K << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_KSXT :
        if (echoMap[msgType]) {
            EUNIKSXTData *tmpPtr = gnssMsg.xNMEAData.pxEUNIKSXTData;
            std::cout << tmpPtr->utc_time << ' ' << tmpPtr->lon << ' ' << tmpPtr->lat << ' '
                << tmpPtr->height << ' ' << tmpPtr->track_true << ' ' << tmpPtr->vel << ' ' << (int)tmpPtr-> pos_qual 
                << ' ' << unsigned(tmpPtr->heading_qual) << ' ' << unsigned(tmpPtr->hsolnSVs) << ' ' << (int)tmpPtr->msolnSVs << std::endl;
        }
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_OBSVM :
        if (echoMap[msgType]) {
            EUNIOBSVMData *tmpPtr = gnssMsg.xEUNIData.pxEUNIOBSVMData;
            for (size_t i = 0; i < (tmpPtr->obsNum) ; i++) {
                std::cout << std::time(nullptr) << ' ' << tmpPtr->obsNum << ' '
                    << unsigned(tmpPtr->obsSVInfo[i].sysFreq) << ' ' << tmpPtr->obsSVInfo[i].prn << ' '
                    << tmpPtr->obsSVInfo[i].psr << ' ' << tmpPtr->obsSVInfo[i].adr << ' '
                    << tmpPtr->obsSVInfo[i].psr_std << ' ' << tmpPtr->obsSVInfo[i].adr_std << ' '
                    << tmpPtr->obsSVInfo[i].dopp << ' ' << unsigned(tmpPtr->obsSVInfo[i].CN0) << ' '
                    << tmpPtr->obsSVInfo[i].locktime << ' ' << tmpPtr->obsSVInfo[i].trStatus << std::endl;
            }
        }
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_STATECEF :
        if (echoMap[msgType]) {
            _gnssFrame[_gnssFrameLen] = 0;
            std::cout << _gnssFrame << std::endl;
            EUNISATECEFData *tmpPtr = gnssMsg.xEUNIData.pxEUNISATECEFData;
            for (size_t i = 0; i < (tmpPtr->satNum) ; i++) {
                std::cout << std::time(nullptr) << ' ' << tmpPtr->satNum << ' '
                    << tmpPtr->satECEFInfo[i].GNSS_SYS << ' ' << tmpPtr->satECEFInfo[i].prn << ' '
                    << tmpPtr->satECEFInfo[i].satCoordX << ' ' << tmpPtr->satECEFInfo[i].satCoordY << ' '
                    << tmpPtr->satECEFInfo[i].satCoordZ << ' ' << tmpPtr->satECEFInfo[i].satClk << ' '
                    << tmpPtr->satECEFInfo[i].lonoDly << ' ' << tmpPtr->satECEFInfo[i].tropDly << std::endl;
            }
        }
        ret = true;
        break;
    default:
        ret = false;
        break;
    }
    return ret;
}

bool GNSS::getGNSSDataPtr(const GNSSMSGTYPE &msgType, void*& msgPtr)
{
    bool ret = false;
    switch (msgType)
    {
    case GNSSMSGTYPE::NMEA_XXZDA :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAZDAData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGGA :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGGAData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXTHS :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEATHSData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGNS :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGNSData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXRMC :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEARMCData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGST :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGSTData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGSA :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGSAData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXGSV :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGSVData);
        ret = true;
        break;
    case GNSSMSGTYPE::NMEA_XXVTG :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAVTGData);
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_KSXT :
        msgPtr = static_cast<void*>(gnssMsg.xNMEAData.pxEUNIKSXTData);
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_OBSVM :
        msgPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNIOBSVMData);
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_STATECEF :
        msgPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNISATECEFData);
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_STADOP :
        msgPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNISATDOPData);
        ret = true;
        break;
    case GNSSMSGTYPE::EUNI_PVTSLN :
        msgPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNIPVTSLNData);
        ret = true;
        break;
    default:
        ret = false;
        break;
    }
    return ret;
}

bool GNSS::getGNSSFreq(GNSSSYSTYPE gnssType, uint32_t freqType, double &freq) 
{
    bool ret = false;
    switch (gnssType)
    {
    case GNSSSYSTYPE::GPS:
        switch ((GNSSGPSCHL)freqType)
        {
        case GNSSGPSCHL::L1CA:
        case GNSSGPSCHL::L1CD:
            freq = 1575.42;
            ret = true;
        case GNSSGPSCHL::L1CP:
            freq = 1575.42;
            break;
        case GNSSGPSCHL::L2C:
        case GNSSGPSCHL::L2PY:
            freq = 1227.60;
            break;
        case GNSSGPSCHL::L5D:
            freq = 1176.45;
            ret = true;
            break;
        case GNSSGPSCHL::L5P:
            freq = 1176.45;
            break;
        default:
            freq = 0.0;
            break;
        }
        break;
    case GNSSSYSTYPE::GLONASS:
        switch ((GNSSGLCHL)freqType)
        {
        case GNSSGLCHL::L1CA:
            freq = 1602.0;
            ret = true;
            break;
        case GNSSGLCHL::L2CA:
            freq = 1246.0;
            ret = true;
            break;
        default:
            freq = 0.0;
            ret = false;
            break;
        }
        break;
    case GNSSSYSTYPE::SBAS:
        switch ((GNSSSBASCHL)freqType)
        {
        case GNSSSBASCHL::L1CA:
            freq = 1575.42;
            ret = true;
            break;
        case GNSSSBASCHL::L5:
            freq = 1176.45;
            ret = true;
            break;
        default:
            freq = 0.0;
            break;
        }
        break;
    case GNSSSYSTYPE::GALILEIO:
        switch ((GNSSGALCHL)freqType)
        {
        case GNSSGALCHL::E1B:
        case GNSSGALCHL::E1C:
            freq = 1575.42;
            ret = true;
            break;
        case GNSSGALCHL::E5AP:
            freq = 1176.45;
            break;
        case GNSSGALCHL::E5BP:
            freq = 1207.14;
            break;
        case GNSSGALCHL::E6B:
        case GNSSGALCHL::E6C:
            freq = 1278.75;
            ret = true;
            break;
        default:
            freq = 0.0;
            break;
        }
        break;
    case GNSSSYSTYPE::BDS:
        switch ((GNSSBDSCHL)freqType)
        {
        case GNSSBDSCHL::B1I:
        case GNSSBDSCHL::B1Q:
        case GNSSBDSCHL::B1CD:
            freq  = 1561.098;
            ret = true;
            break;
        case GNSSBDSCHL::B1CP:
            freq  = 1561.098;
            break;
        case GNSSBDSCHL::B2Q:
        case GNSSBDSCHL::B2AD:
        case GNSSBDSCHL::B2B:
            freq  = 1207.14;
            ret = true;
            break;
        case GNSSBDSCHL::B2AP:
            freq  = 1207.14;
            break;
        case GNSSBDSCHL::B3Q:
        case GNSSBDSCHL::B3I:
            freq  = 1268.52;
            ret = true;
            break;
        default:
            freq  = 0.0;
            break;
        }
        break;
    case GNSSSYSTYPE::QZSS:
        switch ((GNSSQZSSCHL)freqType)
        {
        case GNSSQZSSCHL::L1CA:
        case GNSSQZSSCHL::L1CD:
            freq  = 1575.42;
            ret = true;
            break;
        case GNSSQZSSCHL::L1CP:
            freq  = 1575.42;
            break;
        case GNSSQZSSCHL::L5D:
            freq  = 1176.45;
            ret = true;
            break;
        case GNSSQZSSCHL::L5P:
            freq  = 1176.45;
            break;
        case GNSSQZSSCHL::L2CL:
            freq  = 1227.60;
            ret = true;
            break;
        default:
            freq  = 0.0;
            break;
        }
        break;
    case GNSSSYSTYPE::NAVIC:
        switch ((GNSSIRNSSCHL)freqType)
        {
        case GNSSIRNSSCHL::L1D:
            freq  = 1575.42;
            ret = true;
            break;
        case GNSSIRNSSCHL::L5P:
            freq  = 1176.45;
            ret = true;
            break;
        default:
            freq  = 0.0;
            break;
        }
        break;
    default:
        freq = 0.0;
        ret = false;
        break;
    }
    return ret;
}

bool GNSS::getGNSSData(const GNSSMSGTYPE &msgType, void *msgPtr)
{
    bool ret = false;
    bool isOldVal = true;
    void *tmpPtr = nullptr;
    uint32_t memSize;
    if (msgPtr == nullptr) {
        return false;
    }
    
    if (_mtx.try_lock())
    {
        switch (msgType)
        {
        case GNSSMSGTYPE::NMEA_XXZDA :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAZDAData);
            memSize = sizeof(NMEAZDAData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGGA :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGGAData);
            memSize = sizeof(NMEAGGAData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXTHS :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEATHSData);
            memSize = sizeof(NMEATHSData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGNS :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGNSData);
            memSize = sizeof(NMEAGNSData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXRMC :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEARMCData);
            memSize = sizeof(NMEARMCData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGST :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGSTData);
            memSize = sizeof(NMEAGSTData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGSA :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGSAData);
            memSize = sizeof(NMEAGSAData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXGSV :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAGSVData);
            memSize = sizeof(NMEAGSVData);
            ret = true;
            break;
        case GNSSMSGTYPE::NMEA_XXVTG :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxNMEAVTGData);
            memSize = sizeof(NMEAVTGData);
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_KSXT :
            tmpPtr = static_cast<void*>(gnssMsg.xNMEAData.pxEUNIKSXTData);
            memSize = sizeof(EUNIKSXTData);
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_OBSVM :
            tmpPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNIOBSVMData);
            memSize = sizeof(EUNIOBSVMData);
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_STATECEF :
            tmpPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNISATECEFData);
            memSize = sizeof(EUNISATECEFData);
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_STADOP :
            tmpPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNISATDOPData);
            memSize = sizeof(EUNISATDOPData);
            ret = true;
            break;
        case GNSSMSGTYPE::EUNI_PVTSLN :
            tmpPtr = static_cast<void*>(gnssMsg.xEUNIData.pxEUNIPVTSLNData);
            memSize = sizeof(EUNIPVTSLNData);
            ret = true;
            break;
        default:
            ret = false;
            break;
        }

        memcpy(msgPtr, tmpPtr, memSize);

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

int GNSS::listenInfLoop()
{
    uint8_t gnssRecvChar;
    bool ret = false;
    
    std::time_t timeEndPoint = std::time(nullptr) + _revWaitTime;
    while (true)
    {
        int iRet = gnssPort.readData((char *)&gnssRecvChar, sizeof(gnssRecvChar));

        if (iRet < 0) {
            /* code */
            return iRet;

        } else if (iRet != 0) {
            ret = _buf.pushIn(gnssRecvChar);
            if (ret != true) {
                std::cout<< "full"<< std::endl;
                return -1;
            }
        } 
        ret = parseUM982();
        if (ret == true) {
            _mtx.lock();
            iRet = (int)handledMessage();
            _mtx.unlock();
            // if (ret == true) {
                // 
            if (isNeedLog) {
                ret = logMsg((GNSSMSGTYPE)iRet);
            }
            if (isNeedEcho) {
                ret = echoMsg((GNSSMSGTYPE)iRet);
            }
                // return iRet;
            // }
            // feed watchdog
            timeEndPoint = std::time(nullptr) + _revWaitTime;
        }

        if (timeEndPoint < std::time(nullptr) ) {
            // imuPort.closePort();
            return -1;
        }
        
    }
}

int GNSS::listen(GNSSMSGTYPE &msgType, void*& msgPtr)
{
    uint8_t gnssRecvChar;
    bool ret = false;
    
    std::time_t timeEndPoint = std::time(nullptr) + _revWaitTime;
    while (true)
    {
        int iRet = gnssPort.readData((char *)&gnssRecvChar, sizeof(gnssRecvChar));

        if (iRet < 0) {
            /* code */
            return iRet;

        } else if (iRet != 0) {
            ret = _buf.pushIn(gnssRecvChar);
            if (ret != true) {
                std::cout<< "full"<< std::endl;
                return -1;
            }
        } 
        ret = parseUM982();
        if (ret == true) {
            _mtx.lock();
            iRet = (int)handledMessage();
            _mtx.unlock();
            // if (ret == true) {
                // 
            if (isNeedLog) {
                ret = logMsg((GNSSMSGTYPE)iRet);
            }
            if (isNeedEcho) {
                ret = echoMsg((GNSSMSGTYPE)iRet);
            }
            getGNSSDataPtr((GNSSMSGTYPE)iRet, msgPtr);
            msgType = (GNSSMSGTYPE)iRet;
            return ret;
            // }
        }

        if (timeEndPoint < std::time(nullptr) ) {
            // imuPort.closePort();
            return -1;
        }
        
    }
}

bool GNSS::changeWaitTime(uint32_t delayInS)
{
    _revWaitTime = delayInS;
    return true;
}

bool GNSS::isOpen()
{
    return isPortOpen;
}
