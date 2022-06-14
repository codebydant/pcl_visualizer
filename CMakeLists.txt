# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# set project name
project(pcl-visualizer VERSION 1.0.0
    DESCRIPTION "Point cloud visualizser with PCL"
)

# set build type = Release mode
set(CMAKE_BUILD_TYPE Debug)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/color.cmake")

message("\n" "${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} ")
message("${BoldCyan}=========================================")

# set cmake for use std c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")

# #############################################################################
# PACKAGES DEPENDENCIES
# #############################################################################
message("${BoldYellow}***********************")
message("${BoldYellow}PCL PACKAGE")
message("${BoldYellow}***********************")

find_package(PCL 1.8 REQUIRED QUIET)

if(PCL_FOUND)
    message(STATUS "${BoldGreen}PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
else()
    message(FATAL_ERROR "${BoldRed}    ERROR: PCL minimum required version 1.8. Not found")
endif()

message("${BoldYellow}***********************")
message("${BoldYellow}VTK PACKAGE")
message("${BoldYellow}***********************")

find_package(VTK 8.1 REQUIRED QUIET)

if(VTK_FOUND)
    message(STATUS "${BoldGreen}VTK status:")
    message(STATUS "    version: ${VTK_VERSION}")
    message(STATUS "    directory: ${VTK_DIR}")
else()
    message(FATAL_ERROR "${BoldRed} ERROR: VTK minimum required version 8.1. Not found")
endif()

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

# #############################################################################
# ADD LIBRARIES FOLDER
# #############################################################################
add_subdirectory(parser)

# #############################################################################
# SOURCE CODE
# #############################################################################
set(MAIN_SOURCE "src/main.cpp")

# #############################################################################
# EXECUTABLES
# #############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

# #############################################################################
# TARGET LIBRARIES
# #############################################################################
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
target_include_directories(cloud_parser PRIVATE ${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} cloud_parser)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp -Wall -Wextra)

message("${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("${BoldCyan}=========================================")

# #############################################################################
# INSTALL DIRECTORY
# #############################################################################
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

option(BUILD_TESTING "Builds only the test executable." OFF)
option(CODE_COVERAGE "Collect coverage from test library" OFF)

if(BUILD_TESTING)
    message("${BoldWhite} Testing enabled ${ColourReset}")
    enable_testing()
    add_subdirectory(external)
    add_subdirectory(tests)

    if(CODE_COVERAGE)
        message("${BoldWhite} Coverage enabled ${ColourReset}")
        include("${CMAKE_CURRENT_LIST_DIR}/cmake/CodeCoverage.cmake")
        append_coverage_compiler_flags()
        setup_target_for_coverage_lcov(NAME coverage EXECUTABLE ctest -j ${n_cores}
            BASE_DIRECTORY ../coverage
            DEPENDENCIES
            testlib
            testconcreteparses
        )
    endif()
endif()