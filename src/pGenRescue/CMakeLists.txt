#--------------------------------------------------------
# The CMakeLists.txt for:                       pGenRescue
# Author(s):                              Kevin Becker
#--------------------------------------------------------

SET(SRC
  GenRescue.cpp
  GenRescue_Info.cpp
  main.cpp
  gr_utils.cpp
)

ADD_EXECUTABLE(pGenRescue ${SRC})

TARGET_LINK_LIBRARIES(pGenRescue
   ${MOOS_LIBRARIES}
   contacts
   geometry
   apputil
   mbutil
   m
   pthread)

