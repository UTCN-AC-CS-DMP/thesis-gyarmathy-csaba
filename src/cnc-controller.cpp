#include "cnc-controller.hh"
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>

CNCController::CNCController() 
    : serial_port(nullptr), current_line_index(0), streaming(false) {}

CNCController::~CNCController() {
    stop();
}

void CNCController::setPortName(const std::string& portname) {
    port_name = portname;
    openSerialPort();
}

void CNCController::openSerialPort() {
    if (serial_port) {
        closeSerialPort();
    }
    serial_port = new boost::asio::serial_port(io_service, port_name);
    serial_port->set_option(boost::asio::serial_port_base::baud_rate(9600));
}

void CNCController::closeSerialPort() {
    if (serial_port) {
        serial_port->close();
        delete serial_port;
        serial_port = nullptr;
    }
}

void CNCController::loadGCode(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open G-code file: " << filepath << std::endl;
        return;
    }

    gcode_lines.clear();
    std::string line;
    while (std::getline(file, line)) {
        gcode_lines.push_back(line);
    }

    current_line_index = 0;
    file.close();
    std::cout << "G-code file loaded: " << filepath << std::endl;
}

void CNCController::stream() {
    if (!streaming) {
        streaming = true;
        std::cout << "Starting CNC operation..." << std::endl;
    }

    for (current_line_index = 0; current_line_index < gcode_lines.size(); ++current_line_index) {
        sendGCodeLine(gcode_lines[current_line_index]);
    }

    streaming = false;
    std::cout << "CNC operation completed." << std::endl;
}

void CNCController::sendGCodeLine(const std::string& line) {
    if (!serial_port) {
        std::cerr << "Serial port is not open." << std::endl;
        return;
    }

    boost::asio::write(*serial_port, boost::asio::buffer(line + '\n'));
    std::cout << "Sent: " << line << std::endl;
}

void CNCController::returnHome() {
    if (!serial_port) {
        std::cerr << "Serial port is not open." << std::endl;
        return;
    }

    const std::vector<std::string> homeCommands = {
        "M300 S50.00 (pen up)",
        "G4 P150 (wait 150ms)",
        "M300 S255 (turn off servo)",
        "G1 X0 Y0 F3500.00",
        "G1 Z0.00 F150.00 (go up to finished level)",
        "G1 X0.00 Y0.00 F3500.00 (go home)",
        "M18 (drives off)"
    };

    for (const auto& cmd : homeCommands) {
        sendGCodeLine(cmd);
    }

    std::cout << "CNC returned home." << std::endl;
}

void CNCController::stop() {
    streaming = false;
    closeSerialPort();
    std::cout << "CNC operation stopped and serial port closed." << std::endl;
}
