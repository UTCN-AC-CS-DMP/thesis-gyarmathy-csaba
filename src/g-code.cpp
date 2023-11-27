#include "g-code.hh"

void GCode::generate_gcode(const std::vector<std::pair<int, int>> &boundary,
                           std::string filename) {
  std::ofstream file(filename);

  // G-code header
  file << "G90 ; Set to absolute positioning mode\n";
  file << "G21 ; Set units to millimeters\n";
  file << "G28 ; Home all axes\n";
  file << "\n";

  // Set starting point as G00 command
  file << "G00 X" << boundary[0].second << " Y" << boundary[0].first
       << std::endl;

  // Generate G01 commands for boundary scanning
  for (size_t i = 1; i < boundary.size(); i++) {
    file << "G01 X" << boundary[i].second << " Y" << boundary[i].first
         << std::endl;
  }

  // G-code footer
  file << "\n";
  file << "M2 ; End of program\n";

  std::cout << "G-code file generated: " << filename << std::endl;

  file.close();
}