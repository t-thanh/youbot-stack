SET(OROCOS_OODL_INCLUDE_DIR
	${COMMON_INCLUDE_DIR}	
	${SOEM_INCLUDE_DIR}
	${OODL_YOUBOT_INCLUDE_DIR}
)

SET(OROCOS_OODL_libs
	${SOEM_LIBRARIES}
	${OODL_YOUBOT_LIBRARIES}
	boost_thread
	YouBot_OODL_plugin
)

SET(OROCOS_OODL_FILES YouBotArmService.cpp YouBotBaseService.cpp YouBotOODL.cpp)

INCLUDE_DIRECTORIES(${OROCOS_OODL_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_component(Orocos_OODL ${OROCOS_OODL_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(Orocos_OODL ${OROCOS_OODL_libs})

set_target_properties(Orocos_OODL PROPERTIES COMPILE_FLAGS "-Wall")

orocos_install_headers( YouBotTypes.h )

#
# Plugin
#

#defines where static libraries are stored
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/plugins)

#defines where dynamic libraries (plugins) are stored
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin/plugins)

orocos_plugin(YouBot_OODL_plugin YouBotHelpersPlugin.cpp YouBotHelpers.cpp ${COMMON_INCLUDE_DIR})


#orocos_component(${PROJECT_NAME} src/youbot_master_component.cpp src/youbot_base_service.cpp src/youbot_arm_service.cpp src/youbot_helpers.cpp)
#target_link_libraries(${PROJECT_NAME} rtt-ros-youbot_master_rtt-typekit)
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-Wall -Werror")

#orocos_plugin(youbot-plugin src/youbot_plugin.cpp)
