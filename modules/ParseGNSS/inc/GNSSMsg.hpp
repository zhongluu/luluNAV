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
#ifndef _GNSSMSG_
#define _GNSSMSG_

#include <cstdint>
#include <vector>

/**********************ENUM MSG TYPE********************/

enum class GNSSMSGTYPE {
    NMEA_XXZDA,
    NMEA_XXGGA,
    NMEA_XXTHS,
    NMEA_XXGNS,
    NMEA_XXRMC,
    NMEA_XXGST,
    NMEA_XXGSA,
    NMEA_XXGSV,
    NMEA_XXVTG,
    EUNI_KSXT,
    EUNI_OBSVM,
    EUNI_STATECEF,
    EUNI_EPH,
    EUNI_PVTSLN,
    EUNI_STADOP,
    UNDEFINED,
    FAILURE
};

#define EUNI_STATECEF_GPSCHARSUM ('G' + 'P' + 'S')
#define EUNI_STATECEF_BDSCHARSUM ('B' + 'E' + 'I' + 'D' + 'O' + 'U')
#define EUNI_STATECEF_GLONASSCHARSUM ('G' + 'L' + 'O' + 'N' + 'A' + 'S' + 'S')
#define EUNI_STATECEF_SBASCHARSUM ('S' + 'B' + 'A' + 'S')
#define EUNI_STATECEF_GALILEOCHARSUM ('G' + 'A' + 'L' + 'I' + 'L' + 'E' + 'O')
#define EUNI_STATECEF_QZSSCHARSUM ('Q' + 'Z' + 'S' + 'S')
#define EUNI_STATECEF_NAVICCHARSUM ('N' + 'A' + 'V' + 'I' + 'C')

enum class GNSSSYSTYPE {
    GPS,
    GLONASS,
    SBAS,
    GALILEIO,
    BDS,
    QZSS,
    NAVIC,
    UNDEFINED
};

enum class GNSSGPSCHL {
    L1CA = 0,
    L1CP = 3,
    L5D = 6,
    L2PY = 9,
    L1CD = 11,
    L5P = 14,
    L2C = 17,
    UNDEFINED
};

enum class GNSSBDSCHL {
    B1I = 0,
    B1Q = 4,
    B2Q = 5,
    B3Q = 6,
    B1CP = 8,
    B2AP = 12,
    B2B = 13,
    B2I = 17,
    B3I = 21,
    B1CD = 23,
    B2AD = 28,
    UNDEFINED
};

enum class GNSSGLCHL {
    L1CA = 0,
    L2CA = 5,
    G3I = 6,
    G3Q = 7,
    UNDEFINED
};

enum class GNSSGALCHL {
    E1B = 1,
    E1C = 2,
    E5AP = 12,
    E5BP = 17,
    E6B = 18,
    E6C = 22,
    UNDEFINED
};

enum class GNSSQZSSCHL {
    L1CA = 0,
    L1CP = 3,
    L5D = 6,
    L1CD = 11,
    L5P = 14,
    L2CL = 17,
    UNDEFINED
};

enum class GNSSSBASCHL {
    L1CA = 0,
    L5 = 6,
    UNDEFINED
};

enum class GNSSIRNSSCHL {
    L1D = 6,
    L5P = 14,
    UNDEFINED
};

/**********************STD NMEA MSG********************/

typedef struct tagNMEAZDAData
{
    double utc_time = 0.0;
    uint8_t day = 0;
    uint8_t month = 0;
    uint16_t year = 0;
    uint8_t local_time_off_hour = 0;
    uint8_t local_time_off_min = 0;
} NMEAZDAData;

typedef struct tagNMEAGGAData
{
    double utc_time = 0.0;
    double lat = 0.0;
    char ns = '\0';
    double lon = 0.0;
    char ew = '\0';
    uint8_t fix_quality = 0;
    uint8_t num_of_sv = 0;
    float hdop = 0.0;
    float alt = 0.0;
    float geoid_h = 0.0;
    float dgps_age = 0.0;
} NMEAGGAData;

typedef struct tagNMEATHSData
{
    float heading_deg = 0.0;
} NMEATHSData;

typedef struct tagNMEAGNSData
{
    double utc_time = 0.0;
    double lat = 0.0;
    char ns = 0;
    double lon = 0.0;
    char ew = 0;
    char pos_Mode[6] = {0};
    uint8_t num_of_sv = 0;
    float hdop = 0.0;
    float alt = 0.0;
} NMEAGNSData;

typedef struct tagNMEARMCData
{
    double utc_time = 0.0;
    char Status = 0;
    double lat = 0.0;
    char ns = 0;
    double lon = 0.0;
    char ew = 0;
    float ground_speed_K = 0.0;
    float track_true = 0.0;
    int nmea_date = 0;
    float Mag_var = 0.0;
} NMEARMCData;

typedef struct tagNMEAGSTData
{
    double utc_time = 0.0;
    float rms_err = 0.0;
    float maj_err = 0.0;
    float min_err = 0.0;
    float deg_from_north = 0.0;
    float lat_err = 0.0;
    float lon_err = 0.0;
    float alt_err = 0.0;
} NMEAGSTData;

typedef struct tagNMEAGSAData
{
    char M_pos = 0;
    uint8_t fix_mode = 0;
    uint8_t sat_id[12] = {0};
    float pdop = 0.0;
    float hdop = 0.0;
    float vdop = 0.0;
} NMEAGSAData;

typedef struct tagNMEAGSVSAT
{
    uint8_t svid = 0;
    uint8_t elevation = 0;
    uint16_t azimuth = 0;
    uint8_t snr = 0;
} NMEAGSVSAT;

typedef struct tagNMEAGSVData
{
    GNSSSYSTYPE gnsstype = GNSSSYSTYPE::UNDEFINED;
    uint8_t all_page_num = 0;
    uint8_t this_page_num = 0;
    uint8_t tot_sv_visible = 0;
    NMEAGSVSAT xSat[4];
    uint8_t signalID = 0;
} NMEAGSVData;

typedef struct tagNMEAVTGData
{
    float track_true = 0.0;
    char T = 0;
    float track_mag = 0.0;
    char M = 0;
    float ground_speed = 0.0;
    char N = 0;
    float ground_speed_K = 0.0;
    char K = 0;
} NMEAVTGData;

/****************Extended Unicore MSG********************/

typedef struct tagEUNIKSXTData
{
    double utc_time = 0.0;
    double lon = 0.0; // (deg)
    double lat = 0.0; // (deg)
    double height = 0.0; // (m)
    float heading = 0.0; // (deg)
    float pitch = 0.0; // (deg)
    float track_true = 0.0; // (deg)
    float vel = 0.0; // (km/h)
    float roll = 0.0; // (deg)
    uint8_t pos_qual = 0;
    uint8_t heading_qual = 0;
    uint8_t hsolnSVs = 0; //  
    uint8_t msolnSVs = 0; // master ant sv
    float posEast = 0.0; // based on reference station in geographic coordinate system
    float posNorth = 0.0;
    float posUp = 0.0;
    float velEast = 0.0; // in geographic coordinate system
    float velNorth = 0.0;
    float velUp = 0.0;
} EUNIKSXTData;

typedef struct tagEUNIHEADERData
{
    uint8_t cpuDle = 0;
    char timeSys[5] = {0};
    char timeStatus[9] = {0};
    uint16_t wn = 0;
    uint32_t ms = 0;
    uint8_t leapSec = 0;
    uint16_t outDly = 0;
} EUNIHEADERData;

typedef struct tagEUNIECEFSAT
{
    char GNSS_SYS[16] = {0};
    uint32_t sysType = 0;
    uint16_t prn = 0;
    float satCoordX = 0.0; // (m)
    float satCoordY = 0.0; // (m)
    float satCoordZ = 0.0; // (m)
    float satClk = 0.0; // (m)
    float lonoDly = 0.0; // (m)
    float tropDly = 0.0; // (m)
} EUNIECEFSAT;

typedef struct tagEUNISATECEFData
{
    EUNIHEADERData xHeader;
    uint32_t satNum = 0;
    std::vector<EUNIECEFSAT> satECEFInfo;
}EUNISATECEFData;


#define EUNI_OBSVM_TRSTATUS_ADR_VALID_MASK (0x00000400)
#define EUNI_OBSVM_TRSTATUS_PSR_VALID_MASK (0x00001000)
#define EUNI_OBSVM_TRSTATUS_PSR_ADR_VALID_MASK (0x00001400)
#define EUNI_OBSVM_TRSTATUS_SYS_MASK (0x00070000)
#define EUNI_OBSVM_TRSTATUS_CHL_MASK (0x03E00000)

typedef struct tagEUNISATOBSV
{
    uint8_t sysFreq = 0; // GLONASS frequency unused for GPS, BDS, Galileo
    uint16_t prn = 0;
    double psr = 0.0; // psesodu code (m)
    double adr = 0.0; // carrier
    uint16_t psr_std = 0; // std*100
    uint16_t adr_std = 0; // std*10000
    float dopp = 0.0; // instantaneous doppler (Hz)
    uint16_t CN0 = 0; // snr CN0 = 10*log10(s/n0) db-Hz CN0 * 100
    float locktime = 0.0;
    uint32_t trStatus = 0;
} EUNISATOBSV;


typedef struct tagEUNIOBSVMData
{
    EUNIHEADERData xHeader;
    uint32_t obsNum = 0;
    std::vector<EUNISATOBSV> obsSVInfo;
} EUNIOBSVMData;


typedef struct tagEUNISATDOPData
{
    EUNIHEADERData xHeader;
    uint32_t Itow = 0;
    float gdop = 0;
    float Pdop = 0;
    float Tdop = 0;
    float Vdop = 0;
    float Hdop = 0;
    float Ndop = 0;
    float Edop = 0;
    float cutoff = 0;
    uint16_t PRNnums = 0;
    std::vector<uint16_t> prns;
} EUNISATDOPData;


enum class GNSSLOCTYPET04
{
    NONE,
    FIXEDPOS,
    FIXEDHEIGHT,
    DOPPLER_VELOCITY = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    SBAS,
    L1_FLOAT = 32,
    IONOFREE_FLOAT,
    NARROW_FLOAT,
    L1_INT = 48,
    WIDE_INT,
    NARROW_INT,
    INS,
    INS_PSRSP,
    INS_PSRDIFF,
    INS_RTKFLOAT,
    INS_RTKFIXED,
    PPP_CONVERGING = 68,
    PPP,
    UNDEFINED
};

#define EUNI_PVTSLN_NONESUM ('N' + 'O' + 'N' + 'E')
#define EUNI_PVTSLN_FIXEDPOSSUM ('F' + 'I' + 'X' + 'E' + 'D' + 'P' + 'O' + 'S')
#define EUNI_PVTSLN_FIXEDHEIGHTSUM ('F'+'I'+'X'+'E'+'D'+'H'+'E'+'I'+'G'+'H'+'T')
#define EUNI_PVTSLN_DOPPLER_VELOCITYSUM ('D'+'O'+'P'+'P'+'L'+'E'+'R'+'_'+'V'+'E'+'L'+'O'+'C'+'I'+'T'+'Y')
#define EUNI_PVTSLN_SINGLESUM ('S' + 'I' + 'N' + 'G' + 'L' + 'E')
#define EUNI_PVTSLN_PSRDIFFSUM ('P' + 'S' + 'R' + 'D' + 'I' + 'F' + 'F')
#define EUNI_PVTSLN_SBASSUM ('S' + 'B' + 'A' + 'S')
#define EUNI_PVTSLN_L1_FLOATUM ('L'+'1'+'_'+'F'+'L'+'O'+'A'+'T')
#define EUNI_PVTSLN_IONOFREE_FLOATSUM ('I'+'O'+'N'+'O'+'F'+'R'+'E'+'E'+'_'+'F'+'L'+'O'+'A'+'T')
#define EUNI_PVTSLN_NARROW_FLOATSUM ('N'+'A'+'R'+'R'+'O'+'W'+'_'+'F'+'L'+'O'+'A'+'T')
#define EUNI_PVTSLN_L1_INTSUM ('L'+'1'+'_'+'I'+'N'+'T')
#define EUNI_PVTSLN_WIDE_INTSUM ('W'+'I'+'D'+'E'+'_'+'I'+'N'+'T')
#define EUNI_PVTSLN_NARROW_INTSUM ('N'+'A'+'R'+'R'+'O'+'W'+'_'+'I'+'N'+'T')
#define EUNI_PVTSLN_INSSUM ('I'+'N'+'S')
#define EUNI_PVTSLN_INS_PSRSPSUM ('I'+'N'+'S'+'_'+'P'+'S'+'R'+'S'+'P')
#define EUNI_PVTSLN_INS_PSRDIFFSUM ('I'+'N'+'S'+'_'+'P'+'S'+'R'+'D'+'I'+'F'+'F')
#define EUNI_PVTSLN_INS_RTKFLOATSUM ('I'+'N'+'S'+'_'+'R'+'T'+'K'+'F'+'L'+'O'+'A'+'T')
#define EUNI_PVTSLN_INS_RTKFIXEDSUM ('I'+'N'+'S'+'_'+'R'+'T'+'K'+'F'+'I'+'X'+'E'+'D')
#define EUNI_PVTSLN_PPP_CONVERGINGSUM ('P'+'P'+'P'+'_'+'C'+'O'+'N'+'V'+'E'+'R'+'G'+'I'+'N'+'G')
#define EUNI_PVTSLN_PPPSUM ('P'+'P'+'P')

enum class GNSSHEADTYPET05
{
    SOL_COMPUTED,
    INSUFFICIENT_OBS,
    NO_CONVERGENCE,
    COV_TRACE=4 ,
    NONE,
    UNDEFINED
};

#define EUNI_PVTSLN_SOL_COMPUTEDSUM ('S'+'O'+'L'+'_'+'C'+'O'+'M'+'P'+'U'+'T'+'E'+'D')
#define EUNI_PVTSLN_INSUFFICIENT_OBSSUM ('I'+'N'+'S'+'U'+'F'+'F'+'I'+'C'+'I'+'E'+'N'+'T'+'_'+'O'+'B'+'S')
#define EUNI_PVTSLN_NO_CONVERGENCESUM ('N'+'O'+'_'+'C'+'O'+'N'+'V'+'E'+'R'+'G'+'E'+'N'+'C'+'E')
#define EUNI_PVTSLN_COV_TRACESUM ('C'+'O'+'V'+'_'+'T'+'R'+'A'+'C'+'E')


typedef struct tagEUNIPVTSLNData
{
    EUNIHEADERData xHeader;
    uint32_t bestpos_type = 0;
    float bestpos_hgt = 0;
    double bestpos_lat = 0;
    double bestpos_lon = 0;
    float bestpos_hgtstd = 0;
    float bestpos_latstd = 0;
    float bestpos_lonstd = 0;
    float bestpos_diffage = 0;
    uint32_t psrpos_type = 0;
    float psrpos_hgt = 0;
    double psrpos_lat = 0;
    double psrpos_lon = 0;
    float undulation = 0;
    uint8_t bestpos_svs = 0;
    uint8_t bestpos_solnsvs = 0;
    uint8_t psrpos_svs = 0;
    uint8_t psrpos_solnsvs = 0;
    double psrvel_north = 0;
    double psrvel_east = 0;
    double psrvel_ground = 0;
    uint32_t heading_type = 0;
    float heading_length = 0;
    float heading_degree = 0;
    float heading_pitch = 0;
    uint8_t heading_trackedsvs = 0;
    uint8_t heading_solnsvs = 0;
    uint8_t heading_ggl1 = 0;
    uint8_t heading_ggl1l2 = 0;
    float gdop = 0;
    float pdop = 0;
    float hdop = 0;
    float htdop = 0;
    float tdop = 0;
    float cutoff = 0;
    uint16_t PRNnums = 0;
    std::vector<uint16_t> prns;
} EUNIPVTSLNData;

/****************Global unicore MSG STRUCT********************/

typedef struct tagNMEADataStruct
{
    bool isZDAupdate = false;
    NMEAZDAData *pxNMEAZDAData = nullptr;
    bool isGGAupdate = false;
    NMEAGGAData *pxNMEAGGAData = nullptr;
    bool isTHSupdate = false;
    NMEATHSData *pxNMEATHSData = nullptr;
    bool isGNSupdate = false;
    NMEAGNSData *pxNMEAGNSData = nullptr;
    bool isRMCupdate = false;
    NMEARMCData *pxNMEARMCData = nullptr;
    bool isGSTupdate = false;
    NMEAGSTData *pxNMEAGSTData = nullptr;
    bool isGSAupdate = false;
    NMEAGSAData *pxNMEAGSAData = nullptr;
    bool isGSVupdate = false;
    NMEAGSVData *pxNMEAGSVData = nullptr;
    bool isVTGupdate = false;
    NMEAVTGData *pxNMEAVTGData = nullptr;
    bool isKSXTupdate = false; // header mark with '$' but is an extend unicore cmd
    EUNIKSXTData *pxEUNIKSXTData = nullptr;
} NMEADataStruct;

typedef struct tagEUNIDataStruct
{
    bool isSATEECEFupdate = false;
    EUNISATECEFData *pxEUNISATECEFData = nullptr;
    bool isOBSVMupdate = false;
    EUNIOBSVMData *pxEUNIOBSVMData = nullptr;
    bool isSATDOPupdate = false;
    EUNISATDOPData *pxEUNISATDOPData = nullptr;
    bool isPVTSLNupdate = false;
    EUNIPVTSLNData *pxEUNIPVTSLNData = nullptr;
}EUNIDataStruct;

typedef struct tagGNSSDataMsg
{
    NMEADataStruct xNMEAData;
    EUNIDataStruct xEUNIData;
} GNSSDataMsg;

#endif
