#pragma once
#include "concrete_parses.hpp"

namespace CloudParserLibrary {

class ParserFactory {
 public:
  ParserFactory() { initializes_factories(); }
  void initializes_factories();
  void register_format(std::string format, InterfaceParser *ptr);
  InterfaceParser *get_parser(const std::string format);

 private:
  std::map<std::string, InterfaceParser *> factories = {};
};

}  // namespace CloudParserLibrary
