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

// #define USINGNOVELMETHOD

#define IMUSIMDATAFILE "../data/tightcouple/imuRAW_1807_I.txt"
#define GNSSSIMDATAFILE "../data/tightcouple/gnssRAW_1807_I.txt"

#ifdef USINGNOVELMETHOD
#define TCINSSIMLOGFILE "../data/SIM/tcVBINSSIMLog_1807_I.txt"
#else
#define TCINSSIMLOGFILE "../data/SIM/tcINSSIMLog_1807_I.txt"
#endif

// #define EVALUATETIME



// #define READFROMFILE

#ifndef READFROMFILE
#define GLV_IMU_SAMPLES 125
#else
#define GLV_IMU_SAMPLES 125
#endif

#endif