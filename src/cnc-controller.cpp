#include "cnc-controller.hh"
#include <fstream>
#include <iostream>

CNCController::CNCController() 
    : streaming(false), i(0) {}

CNCController::~CNCController() {
    if (serial.IsConnected()) {
        serial.stop();
    }
}

void CNCController::selectSerialPort(const std::string& portname) {
    if (serial.IsConnected()) {
        serial.stop();
    }

    if (serial.start(portname)) {
        this->portname = portname;
        std::cout << "Serial port " << portname << " opened successfully." << std::endl;
    } else {
        std::cerr << "Error opening serial port: " << portname << std::endl;
    }
}

void CNCController::loadGCodeFile(const std::string& filePath) {
    gcode.clear();
    i = 0;
    std::ifstream file(filePath);
    std::string line;
    while (std::getline(file, line)) {
        gcode.push_back(line);
    }
    if (!gcode.empty()) {
        streaming = true;
    }
}

void CNCController::start() {
    if (streaming) {
        stream();
    }
}

void CNCController::stopAndReset() {
    returnHome();
    streaming = false;
}

void CNCController::stream() {
    if (!streaming) {
        return;
    }

    while (i < gcode.size()) {
        if (gcode[i].empty()) {
            i++;
        } else {
            std::string gcode_line = gcode[i] + "\n";
            if (serial.WriteData(gcode_line.data(), gcode_line.size())) {
                std::cout << "Sent: " << gcode[i] << std::endl;
                i++;
            } else {
                std::cerr << "Error writing to serial port." << std::endl;
                break;
            }
        }
    }
    streaming = false;
}

void CNCController::returnHome() {
    std::string homeCommand = "M300 S50.00 (pen up)\n"
                              "G4 P150 (wait 150ms)\n"
                              "M300 S255 (turn off servo)\n"
                              "G1 X0 Y0 F3500.00\n"
                              "G1 Z0.00 F150.00 (go up to finished level)\n"
                              "G1 X0.00 Y0.00 F3500.00 (go home)\n"
                              "M18 (drives off)\n";
    if (serial.WriteData(homeCommand.data(), homeCommand.size())) {
        std::cout << "Sent: return home commands" << std::endl;
    } else {
        std::cerr << "Error writing return home commands to serial port." << std::endl;
    }
}
