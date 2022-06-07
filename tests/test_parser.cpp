#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <modern/parser.hpp>

TEST_CASE("Testing register_format and get_parser") {
    CloudParserLibrary::ParserFactory parser_factory;
    CloudParserLibrary::ParserPCD* pcd_parser = new CloudParserLibrary::ParserPCD();

    SECTION("Registering a new parser into the factories map") {
        parser_factory.register_format("PCD", pcd_parser);
        REQUIRE(parser_factory.get_size() == 1);

        SECTION("Searches the container for the factory parser") {
            CloudParserLibrary::InterfaceParser* factory = parser_factory.get_parser("PCD");
            REQUIRE(factory == pcd_parser);
        } 
    }          
}