SET(YOUBOT_MONITORS_FILES YouBotMonitor.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_MONITORS_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_service(YouBot_monitors ${YOUBOT_MONITORS_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_monitors rtt-ros-YouBot_monitors-typekit)

set_target_properties(YouBot_monitors PROPERTIES COMPILE_FLAGS "-Wall")

orocos_install_headers( YouBotMonitor.h )

#defines where static libraries are stored
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/plugins)

#defines where dynamic libraries (plugins) are stored
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin/plugins)

orocos_plugin(YouBot_monitors_plugin YouBotMonitorPlugin.cpp ${COMMON_INCLUDE_DIR})