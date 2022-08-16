# #############################################################################
# CMAKE CONFIGURATION
# #############################################################################
cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/color.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

# set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING "Final installation location.")
set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")

project(pcl-visualizer VERSION 1.0.1
    DESCRIPTION "Point cloud visualizser with PCL"
)

message("\n" "${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} ")
message("${BoldCyan}=========================================")

# #############################################################################
# PACKAGES DEPENDENCIES
# #############################################################################
message("${BoldYellow}***********************")
message("${BoldYellow}PCL PACKAGE")
message("${BoldYellow}***********************")

find_package(PCL 1.8 REQUIRED QUIET COMPONENTS visualization)

if(PCL_FOUND)
    message(STATUS "${BoldGreen}PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directory: ${PCL_DIR}")
else()
    message(FATAL_ERROR "${BoldRed}    ERROR: PCL minimum required version 1.8. Not found")
endif()

fetch_project(
    NAME cloudparse
    URL https://github.com/danielTobon43/cloudparse/archive/v0.2.1.tar.gz
)

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
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} cloudparse)
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

# if(BUILD_TESTING)
# message("${BoldWhite} Testing enabled ${ColourReset}")
# enable_testing()
# add_subdirectory(external)

# add_subdirectory(tests)
# if(CODE_COVERAGE)
# message("${BoldWhite} Coverage enabled ${ColourReset}")
# include("${CMAKE_CURRENT_LIST_DIR}/cmake/CodeCoverage.cmake")
# append_coverage_compiler_flags()
# setup_target_for_coverage_lcov(NAME coverage EXECUTABLE ctest -j ${n_cores}
# BASE_DIRECTORY ../coverage
# DEPENDENCIES
# testlib
# testconcreteparses
# )
# endif()
# endif()