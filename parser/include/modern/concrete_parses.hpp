/*
 * Concrete parsers module
 *
 * it implements a factory to create a parser from a abstract class.
 * This module will provided support for new format extensions.
 */
#pragma once
#ifndef CONCRETE_PARSES_HPP
#define CONCRETE_PARSES_HPP
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

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

class ParserPLY : public InterfaceParser {
 public:
  std::string parser_name = "ParserPLY";
  bool cloud_is_good(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if ((cloud->points.size() > 0) or
        (cloud->points[0].x > 0 && cloud->points[0].y > 0 && cloud->points[0].z > 0)) {
      return true;
    }
    return false;
  }
  void load_cloudfile(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    pcl::io::loadPLYFile(filename, *cloud);
    if (cloud_is_good(cloud)) {
      return;
    }

    pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");

    pcl::PolygonMesh cl;
    pcl::io::loadPolygonFile(filename, cl);
    pcl::fromPCLPointCloud2(cl.cloud, *cloud);
    if (cloud_is_good(cloud)) {
      return;
    }
    pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");

    pcl::PLYReader plyRead;
    plyRead.read(filename, *cloud);
    if (cloud_is_good(cloud)) {
      return;
    }

    pcl::console::print_error("\nError .ply file is not compatible.\n");
    std::exit(-1);
  }
};
}  // namespace CloudParserLibrary
#endif