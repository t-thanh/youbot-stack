SET(YOUBOT_MONITORS_FILES YouBotMonitorService.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_MONITORS_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#Executable component and can also be a library usable for orocos deployment
orocos_service(YouBot_monitors ${YOUBOT_MONITORS_FILES} ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(YouBot_monitors rtt-ros-YouBot_monitors-typekit)

set_target_properties(YouBot_monitors PROPERTIES COMPILE_FLAGS "-Wall")

orocos_install_headers( YouBotMonitorService.h )