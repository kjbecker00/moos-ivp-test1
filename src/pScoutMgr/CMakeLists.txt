#--------------------------------------------------------
# The CMakeLists.txt for:                       pScoutMgr
# Author(s):                              Kevin Becker
#--------------------------------------------------------

SET(SRC
  ScoutMgr.cpp
  ScoutMgr_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pScoutMgr ${SRC})

TARGET_LINK_LIBRARIES(pScoutMgr
   ${MOOS_LIBRARIES}
   contacts
   geometry
   apputil
   mbutil
   m
   pthread)

