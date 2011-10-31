INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR} ${SOEM_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR})

SET(libYouBotOODL_libs
	${SOEM_LIBRARIES}
	${OODL_YOUBOT_LIBRARIES}
	boost_thread
	YouBot_OODL
)

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

orocos_component(ArmControllerMockup ArmControllerMockup.h ArmControllerMockup.cpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(ArmControllerMockup ${libYouBotOODL_libs})

orocos_component(BaseControllerMockup BaseControllerMockup.h BaseControllerMockup.cpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(BaseControllerMockup ${libYouBotOODL_libs})

orocos_component(GripperControllerMockup GripperControllerMockup.h GripperControllerMockup.cpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(GripperControllerMockup ${libYouBotOODL_libs})