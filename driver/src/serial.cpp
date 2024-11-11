#include "serial.hpp"

#include <iostream>
#include <fcntl.h>    // 文件控制定义
#include <unistd.h>   // UNIX标准函数定义
#include <cstring>    // 字符串操作
#include <cerrno>     // 错误码

SerialPort::SerialPort(const std::string &portName, int baudRate, int dataBits, int stopBits, char parity, bool hardwareFlowControl)
    : _portName(portName), _baudRate(baudRate), _dataBits(dataBits), _stopBits(stopBits), _parity(parity), _hardwareFlowControl(hardwareFlowControl), _fd(-1) 
{
    memset(&_tty, 0, sizeof(_tty));
}

SerialPort::~SerialPort() 
{
    closePort();
}

bool SerialPort::openPort() 
{
    _fd = open(_portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (_fd == -1) {
        std::cerr << "Failed to open serial port " << _portName << ": " << strerror(errno) << std::endl;
        return false;
    }
    // 设置为阻塞模式
    fcntl(_fd, F_SETFL, 0);
    return configurePort(_baudRate, _dataBits, _stopBits, _parity, _hardwareFlowControl);
}

void SerialPort::closePort() 
{
    if (_fd != -1) {
        close(_fd);
        _fd = -1;
    }
}

bool SerialPort::configurePort(int baudRate, int dataBits, int stopBits, char parity, bool hardwareFlowControl) 
{
    if (tcgetattr(_fd, &_tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }
    memset(&_tty, 0, sizeof(_tty));
    // 设置波特率
    speed_t speed;
    switch (baudRate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
            std::cerr << "Unsupported baud rate: " << baudRate << std::endl;
            return false;
    }
    cfsetospeed(&_tty, speed);
    cfsetispeed(&_tty, speed);

    // 设置数据位
    _tty.c_cflag &= ~CSIZE;
    switch (dataBits) {
        case 5: _tty.c_cflag |= CS5; break;
        case 6: _tty.c_cflag |= CS6; break;
        case 7: _tty.c_cflag |= CS7; break;
        case 8: _tty.c_cflag |= CS8; break;
        default:
            std::cerr << "Unsupported data bits: " << dataBits << std::endl;
            return false;
    }

    // 设置停止位
    if (stopBits == 1) {
        _tty.c_cflag &= ~CSTOPB;
    } else if (stopBits == 2) {
        _tty.c_cflag |= CSTOPB;
    } else {
        std::cerr << "Unsupported stop bits: " << stopBits << std::endl;
        return false;
    }

    // 设置校验位
    if (parity == 'N') {
        _tty.c_cflag &= ~PARENB;   // 无奇偶校验
    } else if (parity == 'E') {
        _tty.c_cflag |= PARENB;    // 启用奇偶校验
        _tty.c_cflag &= ~PARODD;   // 偶校验
    } else if (parity == 'O') {
        _tty.c_cflag |= PARENB;
        _tty.c_cflag |= PARODD;    // 奇校验
    } else {
        std::cerr << "Unsupported parity: " << parity << std::endl;
        return false;
    }

    // 设置硬件流控制
    if (hardwareFlowControl) {
        _tty.c_cflag |= CRTSCTS;
    } else {
        _tty.c_cflag &= ~CRTSCTS;
    }

    // 其他配置
    _tty.c_cflag |= (CREAD | CLOCAL);  // 使能读取
    _tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 关闭回显、信号处理
    // _tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // 禁用软件流控制
    _tty.c_oflag &= ~OPOST;                           // 禁用输出处理

    tcflush(_fd, TCIFLUSH);
    
    // 应用设置
    if (tcsetattr(_fd, TCSANOW, &_tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void SerialPort::setReadTimeout(int vmin, int vtime) 
{
    _tty.c_cc[VMIN] = vmin;
    _tty.c_cc[VTIME] = vtime;

    // 应用设置
    if (tcsetattr(_fd, TCSANOW, &_tty) != 0) {
        std::cerr << "Error from tcsetattr (setReadTimeout): " << strerror(errno) << std::endl;
    }
}

ssize_t SerialPort::readData(char *buffer, size_t size) 
{
    if (_fd == -1) {
        std::cerr << "Serial port not open." << std::endl;
        return -1;
    }
    return read(_fd, buffer, size);
}

ssize_t SerialPort::readData(char *buffer, size_t size, int timeout_ms) 
{
    if (_fd == -1) {
        std::cerr << "Serial port not open." << std::endl;
        return -1;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(_fd, &readfds);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(_fd + 1, &readfds, NULL, NULL, &timeout);
    if (ret > 0) {
        return read(_fd, buffer, size);
    } else if (ret == 0) {
        // std::cerr << "Read timeout." << std::endl;
        return 0;
    } else {
        // std::cerr << "Error during select: " << strerror(errno) << std::endl;
        return -1;
    }
}

ssize_t SerialPort::writeData(const char *data, size_t size)
{
    if (_fd == -1) {
        std::cerr << "Serial port not open." << std::endl;
        return -1;
    }
    return write(_fd, data, size);
}


