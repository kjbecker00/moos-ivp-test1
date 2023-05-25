#--------------------------------------------------------
# The CMakeLists.txt for:                       pNodeIdentifier
# Author(s):                              Kevin Becker
#--------------------------------------------------------

SET(SRC
  NodeIdentifier.cpp
  NodeIdentifier_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pNodeIdentifier ${SRC})

TARGET_LINK_LIBRARIES(pNodeIdentifier
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

