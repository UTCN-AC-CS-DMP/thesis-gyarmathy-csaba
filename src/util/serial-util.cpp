#include "serial-util.hh"

#include <boost/asio.hpp>
#include <filesystem>
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

std::vector<std::string> listSerialPorts() {
	std::vector<std::string> port_names;
        // Windows implementation
#if defined(_WIN32) || defined(_WIN64)

	for (int i = 1; i <= 256; ++i) {
		std::string port_name = "COM" + std::to_string(i);
		HANDLE h = CreateFile(port_name.c_str(),
			GENERIC_READ | GENERIC_WRITE,
			0,
			nullptr,
			OPEN_EXISTING,
			0, 
			nullptr);

		if (h != INVALID_HANDLE_VALUE) {
			port_names.push_back(port_name);
			CloseHandle(h);
		}
	}
	// #else
	// Linux and MacOS implementation
#endif
	return port_names;
}
