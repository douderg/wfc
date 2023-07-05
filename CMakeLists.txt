cmake_minimum_required(VERSION 3.15)
project (wfc)

add_library (wfc source/wfc.cpp)
target_include_directories (wfc PUBLIC include)
set_target_properties (wfc PROPERTIES CXX_STANDARD 17)


add_executable (test_wfc main.cpp)
set_target_properties (test_wfc PROPERTIES CXX_STANDARD 17)
target_link_libraries (test_wfc PRIVATE wfc)
