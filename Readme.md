# How to Use
## 1 Primary configuration
cd ./src \
sudo -i \
entering the password \
(debian) \
apt-get install libeigen3-dev \
apt-get install gsl-bin libgsl-dev 

## 2 Building
cd /LULUNAV \
mkdir build \
cd ./build \
cmake .. \
make

## 3 Running in root user
sudo -i \
entering the password \
./LuNavigation

## 4 Code configuration in "*./inc/main.hpp*"
1. if you want to use the novel VB - sequential processing code please uncomment the macro "*USINGNOVELMETHOD*".
2. if you want to evaluate the time consumption please uncomment the macro "*EVALUATETIME*".
3. if you want to running the code from saved raw data please uncomment the macro "*READFROMFILE*" and specify the file path by revise the macro "*IMUSIMDATAFILE*" and "*GNSSSIMDATAFILE*". Meanwhile, the output path alse need to specify by revise the macro "*TCINSSIMLOGFILE*".
4. if your IMU frequency is not 125 Hz, please revise the macro "*GLV_IMU_SAMPLES*".
5. change the serial port macro "*IMUUARTPORT*" for IMU and "*GNSSUARTPORT*" for GNSS in real time running (if necessary).
   
## 5 Navigation configuration
please refer to the readme of the "./config/Readme.md"

# Software framework

## Sensors information to Navigation system
like the uORB of PIXhawk framework, the publisher - subscriber pattern is used in this framework with std::thread (C++11)
## Parse sensors packets class 
template Ringbuff class to parse class for thread
## TCINS for tight couple navigation
Eigen lib is used in abstract INS class \
nested ekf class in the TCINS class for the kalman filter \
changing the tight couple navigation inital configuration in '/config/insconfig.txt' with key-value pair

# TODO LIST:
## 1 Factory - product pattern for TCINS
## 2 Factory - product pattern for TCNAV
## 3 Unsubscribe for the publisher - subscriber pattern
## 4 Decorator to the parse class
## 5 DMA for single core CPU
## 6 Support for linux kernel mode
## 7 EPH (ephemeris) data disposition processing for range rate measurement
## 8 Support range rate measurement for TCINS
## 9 Coarse alignment

# NOTE:
1. subscriber must be create by make_shared 
2. A initial attitude must to be config in the "insEXconfig.txt" 
3. The GNSS only support fixed 1 Hz until now
4. If you want to running the Matlab code, the psins toolbox must be applied from  https://www.psins.org.cn/
5. There are slight differences between the result of C++ and the result of Matlab.
6. A GPL license is used in mine code, commercial using is not allowed without permission
7. The corresponding work is submitted to "" with ID "".

# Estimation Result:
## Novel method estimation result
Attitude:

![VBAttitude](./img/VBAtti.jpg "Attitude")

Velocity:

![VBAttitude](./img/VBVel.jpg "Velocity")

Position:

![VBAttitude](./img/VBPos.jpg "Position")

Gyroscope Bias:

![VBAttitude](./img/VBBiasG.jpg "BiasG")

Accelerometer Bias:

![VBAttitude](./img/VBBiasA.jpg "BiasA")

Lever:

![VBAttitude](./img/VBLever.jpg "Lever")

Time delay:

![VBAttitude](./img/VBTimeDly.jpg "delay")

Clock offset and clock drift:

![VBAttitude](./img/VBClockOffsetAndDrift.jpg "VBClockOffsetAndDrift")

trajectory:

![VBAttitude](./img/VBtrj.jpg "trajectory")

R estimation:

![VBAttitude](./img/VBR.jpg "VBR")

# Acknowledgment:
Thanks Dr. Gongmin Yan's open source code psins toolbox from https://www.psins.org.cn/ in Northwestern Polytechnical University of P.R.China.

#
data: 09/11/2024 \
lu email: zhong.yulu.ll@gmail.com or yulu_zhong@seu.edu.cn