#include <modern/parser.hpp>
namespace CloudParserLibrary {

void ParserFactory::register_format(std::string format, InterfaceParser* ptr) {
  // ParserFactory::factories.insert(std::make_pair(format, creator));
  ParserFactory::factories[format] = ptr;
}

void ParserFactory::initializes_factories() {
  ParserFactory::register_format("PCD", new ParserPCD());
  ParserFactory::register_format("PLY", new ParserPLY());
}

InterfaceParser* ParserFactory::get_parser(const std::string format) {
  try {
    InterfaceParser* factory_pos = factories.at(format);
    return factory_pos;
  } catch (const std::out_of_range& e) {
  }
  return nullptr;
}

}  // namespace CloudParserLibrary
