##############################################################################
# LIBRARY NAME
##############################################################################
set(LIBRARY_NAME Catch2)

##############################################################################
# # Header only library, therefore INTERFACE
##############################################################################
add_library(${LIBRARY_NAME} INTERFACE)

##############################################################################
# INTERFACE targets only have INTERFACE properties
##############################################################################
target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_SOURCE_DIR}/external/include)
# set_target_properties(${LIBRARY_NAME} PROPERTIES
#     # VERSION ${PROJECT_VERSION}
#     # SOVERSION 1
#     PUBLIC_HEADER ${CMAKE_SOURCE_DIR}/external/include/Catch2/catch.hpp
# )

##############################################################################
# LIBRARY HEADERS
##############################################################################
# target_include_directories(${LIBRARY_NAME} INTERFACE
#     ${CMAKE_SOURCE_DIR}/external/include
# )

##############################################################################
# LIBRARY DEPENDENCIES
##############################################################################
# target_link_libraries(${LIBRARY_NAME} ${VTK_LIBRARIES})

##############################################################################
# COMPILATION FLAGS
##############################################################################
# target_compile_features(${LIBRARY_NAME} INTERFACE cxx_std_17)
target_compile_options(${LIBRARY_NAME} INTERFACE -Wno-cpp -Wall -Wextra)

##############################################################################
# INSTALL DIRECTORY
##############################################################################
install(TARGETS ${LIBRARY_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)