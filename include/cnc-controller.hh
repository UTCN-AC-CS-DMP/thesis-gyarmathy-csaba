#ifndef CNC_CONTROLLER_HH
#define CNC_CONTROLLER_HH

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <atomic>

class CNCController {
public:
    CNCController();
    ~CNCController();

    void setPortName(const std::string& portname);
    void loadGCode(const std::string& filepath);
    void stream();
    void returnHome();
    void stop();
    void requestStop();

private:
    void openSerialPort();
    void closeSerialPort();
    void sendGCodeLine(const std::string& line);
    std::string readResponse();

    boost::asio::io_service io_service;
    boost::asio::serial_port* serial_port;
    std::string port_name;
    std::vector<std::string> gcode_lines;
    size_t current_line_index;
    std::atomic<bool> streaming;
    std::atomic<bool> stop_requested;
};

#endif // CNC_CONTROLLER_HH
