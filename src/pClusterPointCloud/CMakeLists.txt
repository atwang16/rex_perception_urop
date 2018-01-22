#--------------------------------------------------------
# The CMakeLists.txt for:                       pClusterPointCloud
# Author(s):                              Austin Wang
#--------------------------------------------------------

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

