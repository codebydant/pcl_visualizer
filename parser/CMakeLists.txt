# project(cloud_parser VERSION 1.0.1)
set(LIBRARY_NAME cloud_parser)
add_library(${LIBRARY_NAME} SHARED
    parser.cpp
)

set_target_properties(${LIBRARY_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/parser/include/modern/parser.hpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/parser/include
)

target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_11)

install(TARGETS ${LIBRARY_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)