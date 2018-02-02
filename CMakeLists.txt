cmake_minimum_required(VERSION 3.7)
project(rex_perception_urop)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(MY_SOURCE_DIR "/usr/local")

add_executable(process_pcap src/process_pcap.cpp)

include_directories(${MY_SOURCE_DIR}/include)

find_package(PCL 1.8 REQUIRED)

if( PCL_FOUND )
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(process_pcap ${PCL_LIBRARIES})
endif()
