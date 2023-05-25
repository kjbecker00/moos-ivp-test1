#--------------------------------------------------------
# The CMakeLists.txt for:                   lib_bhv_scout
# Author(s):                                   M.Benjamin
#--------------------------------------------------------
#                                               BHV_Scout
#--------------------------------------------------------
ADD_LIBRARY(BHV_EscapeWindup SHARED 
   BHV_EscapeWindup.cpp)
TARGET_LINK_LIBRARIES(BHV_EscapeWindup
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
)


