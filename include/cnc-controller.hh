#ifndef CNC_CONTROLLER_H
#define CNC_CONTROLLER_H

#include <string>
#include <vector>
#include "serial.hh"  // Include the custom Serial class

class CNCController {
public:
    CNCController();
    ~CNCController();

    void selectSerialPort(const std::string& portname);
    void loadGCodeFile(const std::string& filePath);
    void start();
    void stopAndReset();
    void stream();
    void returnHome();

private:
    std::string portname;
    bool streaming;
    std::vector<std::string> gcode;
    int i;
    Serial serial;
};

#endif // CNC_CONTROLLER_H
