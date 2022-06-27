cmake_minimum_required(VERSION 3.22)
project(controller)

add_compile_options(-Wall -fPIC)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} include )
set(SOURCES src/perception.cpp src/pid.cpp include/pid.h src/car.cpp include/car.h)

add_library(lib SHARED ${SOURCES})
target_link_libraries( lib ${OpenCV_LIBS} )

add_executable(main main.cpp)

target_link_libraries( main lib )
