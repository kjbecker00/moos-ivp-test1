#--------------------------------------------------------
# The CMakeLists.txt for:                       pSwimSearch
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  SwimSearch.cpp
  SwimSearch_Info.cpp
  main.cpp
  grid.cpp
)

ADD_EXECUTABLE(pSwimSearch ${SRC})

TARGET_LINK_LIBRARIES(pSwimSearch
   ${MOOS_LIBRARIES}
   contacts
   geometry 
   apputil
   mbutil
   m
   pthread)

