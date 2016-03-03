cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PCDRegister)

find_package(PCL 1.6 REQUIRED)
find_package(Boost 1.5 COMPONENTS program_options)

FILE(GLOB hdrs "*.h*")
FILE(GLOB srcs "*.cpp")

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp ${srcs} ${hdrs})
target_link_libraries (main ${PCL_LIBRARIES} ${Boost_LIBRARIES})