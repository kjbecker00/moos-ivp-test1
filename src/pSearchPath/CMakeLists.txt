#--------------------------------------------------------
# The CMakeLists.txt for:                       pSearchPath
# Author(s):                              Kevin Becker
#--------------------------------------------------------

SET(SRC
  SearchPath.cpp
  SearchPath_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSearchPath ${SRC})

TARGET_LINK_LIBRARIES(pSearchPath
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

