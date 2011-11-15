SET(YOUBOT_CONFIGURATOR_INCLUDE_DIR
	${COMMON_INCLUDE_DIR}	
	${SOEM_INCLUDE_DIR}
	${OODL_YOUBOT_INCLUDE_DIR}
	${joint_configurator_PACKAGE_PATH}/src
)

SET(YOUBOT_CONFIGURATOR_libs
	${SOEM_LIBRARIES}
	${OODL_YOUBOT_LIBRARIES}
	JointConfigurator
	boost_thread
)

SET(YOUBOT_CONFIGURATOR_FILES YouBot_configurator.hpp YouBot_configurator.cpp)

INCLUDE_DIRECTORIES(${YOUBOT_CONFIGURATOR_INCLUDE_DIR})

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH} ${joint_configurator_PACKAGE_PATH}/lib)

#Executable component and can also be a library usable for orocos deployment
orocos_component(YouBot_configurator ${YOUBOT_CONFIGURATOR_FILES} )

TARGET_LINK_LIBRARIES(YouBot_configurator ${YOUBOT_CONFIGURATOR_libs})

set_target_properties(YouBot_configurator PROPERTIES COMPILE_FLAGS "-Wall")

