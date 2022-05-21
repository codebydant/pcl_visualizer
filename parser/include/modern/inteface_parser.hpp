#pragma once
#include <iostream>
#include <map>
#include <string>

namespace CloudParserLibrary {

class InterfaceParser {
 public:
  virtual void load_cloudfile(std::string filename) {}
};

}  // namespace CloudParserLibrary