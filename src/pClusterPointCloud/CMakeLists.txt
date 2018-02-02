#--------------------------------------------------------
# The CMakeLists.txt for:                       pClusterPointCloud
# Author(s):                              Austin Wang
#--------------------------------------------------------

set(MY_SOURCE_DIR "/usr/local")
include_directories(${MY_SOURCE_DIR}/include)

SET(SRC
  ClusterPointCloud.cpp
  ClusterPointCloud_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pClusterPointCloud ${SRC})

TARGET_LINK_LIBRARIES(pClusterPointCloud
   ${MOOS_LIBRARIES}
   mbutil
   m
   pthread)

find_package(PCL 1.8 REQUIRED)

if( PCL_FOUND )
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    target_link_libraries(pClusterPointCloud ${PCL_LIBRARIES})
endif()

