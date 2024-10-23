# Using
## 1 primary configuration
cd ./src \
change the serial port macro and the log file path (if needed) in '/src/main.cpp'
sudo -i \
entering the password \
(debian) \
apt-get install libeigen3-dev \
apt-get install gsl-bin libgsl-dev 

## 2 building
cd /LULUNAV \
mkdir build \
cd ./build \
cmake .. \
make

## 3 running in root user
sudo -i \
entering the password \
./LuNavigation

# Software framework

## Sensors information to Navigation system
like the uORB of PIXhawk framework, the publisher - subscriber pattern is used in this framework with std::thread (C++11)
## parse sensors packets class 
template Ringbuff class to parse class for thread
## TCINS for tight couple navigation
Eigen lib is used in abstract INS class \
nested ekf class in the TCINS class for the kalman filter \
changing the tight couple navigation inital configuration in '/config/insconfig.txt' with key-value pair

# TODO LIST:
## 1 factory - product pattern for TCINS
## 2 factory - product pattern for TCNAV
## 3 unsubscribe for the publisher - subscriber pattern
## 4 Decorator to the parse class
## 5 DMA for single core CPU
## 6 support for linux kernel mode
## 7 EPH (ephemeris) data disposition processing for range rate measurement
## 8 support range rate measurement for TCINS

# NOTE:
1, subscriber must be create by make_shared

data: 09/11/2024 \
lu email: zhong.yulu.ll@gmail.com or yulu_zhong@seu.edu.cn