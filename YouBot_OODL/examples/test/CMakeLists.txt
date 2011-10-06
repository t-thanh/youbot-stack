INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR} ${SOEM_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR})

SET(libOrocosOODL_libs
	${SOEM_LIBRARIES}
	${OODL_YOUBOT_LIBRARIES}
	boost_thread
	Orocos_OODL
)

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

orocos_component(ArmControllerMockup ArmControllerMockup.h ArmControllerMockup.cpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(ArmControllerMockup ${libOrocosOODL_libs})

orocos_component(BaseControllerMockup BaseControllerMockup.h BaseControllerMockup.cpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(BaseControllerMockup ${libOrocosOODL_libs})