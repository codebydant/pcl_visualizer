#pragma once
#include "concrete_parses.hpp"
namespace CloudParserLibrary {

class ParserFactory {
 public:
  ParserFactory() { ParserFactory::register_format("PCD", new ParserPCD()); }
  void register_format(std::string format, InterfaceParser *ptr);
  InterfaceParser *get_parser(const std::string format);

 private:
  std::map<std::string, InterfaceParser *> factories = {};
};

class ParserCloudFile {
 public:
  ParserFactory parser_factory;
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    int position = filename.find_last_of(".");
    std::string extension = filename.substr(position + 1);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::toupper);
    InterfaceParser *cloudparser = parser_factory.get_parser(extension);
    pcl::console::TicToc tt;
    pcl::console::print_highlight("Loading ");
    cloudparser->load_cloudfile(filename, cloud);
    pcl::console::print_info("\nFound pcd file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->size());
    pcl::console::print_info(" points]\n");
  }
};

}  // namespace CloudParserLibrary
