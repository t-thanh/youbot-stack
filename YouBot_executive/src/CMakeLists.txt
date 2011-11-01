SET(YOUBOT_EXECUTIVE_INCLUDE_DIR
	${COMMON_INCLUDE_DIR}	
)

SET(YOUBOT_EXECUTIVE_libs
	boost_thread
)

SET(YOUBOT_EXECUTIVE_FILES YouBot_executive.cpp macho/Macho.cpp FlowControl.cpp YouBot_configs.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_EXECUTIVE_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_EXECUTIVE ${YOUBOT_EXECUTIVE_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_EXECUTIVE ${YOUBOT_EXECUTIVE_libs})

set_target_properties(YouBot_EXECUTIVE PROPERTIES COMPILE_FLAGS "-Wall")


#
# Plugin
#

#defines where static libraries are stored
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/plugins)

#defines where dynamic libraries (plugins) are stored
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin/plugins)
