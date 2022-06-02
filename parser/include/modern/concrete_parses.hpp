#pragma once
#include <pcl/io/pcd_io.h>

#include "inteface_parser.hpp"
namespace CloudParserLibrary {
class ParserPCD : public InterfaceParser {
 public:
  std::string parser_name = "ParserPCD";
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (pcl::io::loadPCDFile(filename, *cloud) < 0) {
      pcl::console::print_error("Error loading point cloud %s \n", filename.c_str());
      std::exit(-1);
    }
  }
};
}  // namespace CloudParserLibrary