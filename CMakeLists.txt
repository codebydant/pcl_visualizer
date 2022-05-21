##############################################################################
# CMAKE CONFIGURATION
##############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# set project name
project(pcl-visualizer VERSION 1.0.0 
DESCRIPTION "An example project with CMake"
LANGUAGES CXX)

# set build type = Debug mode
set(CMAKE_BUILD_TYPE Debug)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# set the include directive in the same project folder
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set corresponding package directories
set(PCL_DIR /opt/pcl-1.9.1/build)

# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
set (CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

##############################################################################
# PACKAGES
##############################################################################
message("***********************")
message("PCL PACKAGE")
message("***********************")

# find_package(PCL 1.8 PATHS ${PCL_DIR} REQUIRED COMPONENTS common io visualization)
find_package(PCL 1.9 PATHS ${PCL_DIR} QUIET)
if(PCL_FOUND)
    message(STATUS "PCL status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    directorie: ${PCL_DIR}")
else()
    message(WARNING "${Yellow} PCL 1.9 not found, attempting 1.8...${ColourReset}")
    find_package(PCL 1.8 QUIET)
    if(PCL_FOUND)
        message(STATUS "PCL status:")
        message(STATUS "    version: ${PCL_VERSION}")
        message(STATUS "    directorie: ${PCL_DIR}")
    else()
    	message(WARNING "${Yellow} PCL 1.8 not found, attempting 1.7...${ColourReset}")     
        find_package(PCL 1.7 REQUIRED QUIET)
    	if(PCL_FOUND)
        	message(STATUS "PCL status:")
        	message(STATUS "    version: ${PCL_VERSION}")
        	message(STATUS "    directorie: ${PCL_DIR}")
    	else()
        	message(FATAL_ERROR "    ERROR: PCL minimum required version 1.7. Not found")
    	endif()
    endif()
endif()

##############################################################################
# HEADERS
##############################################################################
include_directories(${PCL_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${PCL_LIBRARY_DIRS})

##############################################################################
# ADD LIBRARIES FOLDER
##############################################################################
add_subdirectory(parser)

##############################################################################
# SOURCE CODE
##############################################################################
set(MAIN_SOURCE "src/main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${MAIN_SOURCE})

##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} cloud_parser)
##target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARY})

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")
