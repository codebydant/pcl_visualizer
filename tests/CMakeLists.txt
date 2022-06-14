# Tests need to be added as executables first
add_executable(testlib test_parser.cpp)
add_executable(testconcreteparses test_concrete_parses.cpp)

# I'm using C++17 in the test
target_compile_features(testlib PRIVATE cxx_std_17)
target_compile_features(testconcreteparses PRIVATE cxx_std_17)

# Should be linked to the main library, as well as the Catch2 testing library
target_link_libraries(testlib PRIVATE Catch2 cloud_parser ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(testconcreteparses PRIVATE Catch2 cloud_parser ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# If you register a test, then ctest and make test will run it.
# You can also run examples and check the output, as well.
add_test(NAME testlibtest COMMAND testlib) # Command can be a target
add_test(NAME testconcreteparses COMMAND testconcreteparses WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}") # Command can be a target

# custom target to get more verbose level in test with: make check
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --verbose)
