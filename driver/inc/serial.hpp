#ifndef _SERIAL_
#define _SERIAL_

#include <string>
#include <termios.h> // 终端控制定义

class SerialPort {
public:
    SerialPort(const std::string &portName, int baudRate, int dataBits = 8, int stopBits = 1, char parity = 'N', bool hardwareFlowControl = false);
    ~SerialPort();

    bool openPort();
    void closePort();
    ssize_t readData(char *buffer, size_t size);
    ssize_t readData(char *buffer, size_t size, int timeout_ms);
    ssize_t writeData(const char *data, size_t size);
    bool configurePort(int baudRate, int dataBits, int stopBits, char parity, bool hardwareFlowControl);
    void setReadTimeout(int vmin, int vtime);

private:
    std::string _portName;
    int _baudRate;
    int _dataBits;
    int _stopBits;
    char _parity;
    bool _hardwareFlowControl;
    int _fd;
    struct termios _tty;
};


#endif