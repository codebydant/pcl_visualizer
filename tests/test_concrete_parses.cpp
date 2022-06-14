#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <filesystem>
#include <modern/concrete_parses.hpp>

TEST_CASE("Testing ParserPCD") {
  CloudParserLibrary::ParserPCD* parser = new CloudParserLibrary::ParserPCD();

  std::filesystem::path filespath("cloudfiles");
  std::filesystem::path filename("cloud.pcd");
  std::filesystem::path TESTDATA_FILENAME = filespath / filename;

  SECTION("Loading cloud pcd file with success") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    parser->load_cloudfile(TESTDATA_FILENAME, cloud);
    REQUIRE((int)cloud->points.size() == 5);
  }
}

TEST_CASE("Testing ParserPLY") {
  CloudParserLibrary::ParserPLY* parser = new CloudParserLibrary::ParserPLY();

  std::filesystem::path filespath("cloudfiles");
  std::filesystem::path filename("cloud.ply");
  std::filesystem::path TESTDATA_FILENAME = filespath / filename;

  SECTION("Loading cloud ply file with success") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    parser->load_cloudfile(TESTDATA_FILENAME, cloud);
    REQUIRE((int)cloud->points.size() == 5);
  }

  SECTION("Testing cloud_is_good function") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    parser->load_cloudfile(TESTDATA_FILENAME, cloud);
    CHECK(parser->cloud_is_good(cloud));
  }
}