#pragma once
#include "inteface_parser.hpp"
namespace CloudParserLibrary {
class ParserPCD : public InterfaceParser {
public:
  std::string parser_name = "ParserPCD";
  void load_cloudfile(std::string filename) {
    std::cout << "pcdparser implementation" << std::endl;
  }
};

class ParserPLY : public InterfaceParser {
public:
  std::string parser_name = "ParserPLY";
  void load_cloudfile(std::string filename) {
    std::cout << "plyparser implementation" << std::endl;
  }
};
} // namespace CloudParserLibrary