#ifndef G_CODE_HH
#define G_CODE_HH

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

class GCode {
 public:
  void generate_gcode(const std::vector<std::pair<int, int>>& boundary,
                      std::string filename);
};

#endif  // G_CODE_HH