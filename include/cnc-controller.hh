#ifndef CNC_CONTROLLER_HH
#define CNC_CONTROLLER_HH

#include <boost/asio.hpp>
#include <string>
#include <vector>

class CNCController {
public:
    CNCController();
    ~CNCController();

    void setPortName(const std::string& portname);
    void loadGCode(const std::string& filepath);
    void stream();
    void returnHome();
    void stop();

private:
    void openSerialPort();
    void closeSerialPort();
    void sendGCodeLine(const std::string& line);

    boost::asio::io_service io_service;
    boost::asio::serial_port* serial_port;
    std::string port_name;
    std::vector<std::string> gcode_lines;
    size_t current_line_index;
    bool streaming;
};

#endif // CNC_CONTROLLER_HH