SET(YOUBOT_OODL_INCLUDE_DIR
	${COMMON_INCLUDE_DIR}	
	${SOEM_INCLUDE_DIR}
	${OODL_YOUBOT_INCLUDE_DIR}
)

SET(YOUBOT_OODL_libs
	${SOEM_LIBRARIES}
	${OODL_YOUBOT_LIBRARIES}
	boost_thread
	YouBot_OODL_plugin
	rtt-ros-YouBot_OODL-typekit
)

SET(YOUBOT_OODL_FILES YouBotArmService.cpp YouBotBaseService.cpp YouBotOODL.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_OODL_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_OODL ${YOUBOT_OODL_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_OODL ${YOUBOT_OODL_libs})

set_target_properties(YouBot_OODL PROPERTIES COMPILE_FLAGS "-Wall")

orocos_install_headers( YouBotTypes.h )

#
# Plugin
#

#defines where static libraries are stored
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/plugins)

#defines where dynamic libraries (plugins) are stored
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin/plugins)

orocos_plugin(YouBot_OODL_plugin YouBotHelpersPlugin.cpp YouBotHelpers.cpp ${COMMON_INCLUDE_DIR})

