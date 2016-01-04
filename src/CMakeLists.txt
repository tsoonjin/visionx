cmake_minimum_required(VERSION 2.8) 
PROJECT (cpp)
find_package(OpenCV REQUIRED)

#=======================================================================================
# Set directories
#======================================================================================
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable( main src/main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )
