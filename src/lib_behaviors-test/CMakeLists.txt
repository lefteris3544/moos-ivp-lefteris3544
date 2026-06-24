#--------------------------------------------------------
# The CMakeLists.txt for:              lib_behaviors-test
# Author(s): Terry Alifragkis                               
#--------------------------------------------------------
#                                      BHV_SimpleWaypoint
#--------------------------------------------------------
ADD_LIBRARY(BHV_SimpleWaypoint SHARED 
   BHV_SimpleWaypoint.cpp AOF_SimpleWaypoint.cpp)
TARGET_LINK_LIBRARIES(BHV_SimpleWaypoint
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry 
   ${SYSTEM_LIBS} )

#--------------------------------------------------------
#                                                BHV_Pulse
#--------------------------------------------------------
ADD_LIBRARY(BHV_Pulse SHARED 
    BHV_Pulse.cpp)
 TARGET_LINK_LIBRARIES(BHV_Pulse
    helmivp
    behaviors 
    ivpbuild 
    logic 
    ivpcore 
    bhvutil 
    mbutil 
    geometry 
    ${SYSTEM_LIBS} )

#--------------------------------------------------------
#                                               BHV_ZigLeg
#--------------------------------------------------------
ADD_LIBRARY(BHV_ZigLeg SHARED
    BHV_ZigLeg.cpp)
TARGET_LINK_LIBRARIES(BHV_ZigLeg
    helmivp
    behaviors
    ivpbuild
    logic
    ivpcore
    bhvutil
    mbutil
    geometry
    ${SYSTEM_LIBS} )
