INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

# Creates a component library libflatToYouBotMsgAddapter-<target>.so
# and installs in the directory lib/orocos/flatToYouBotMsgAddapter/
#
orocos_component(YouBotMsgToTSimAdapter YouBotMsgToTSim.hpp YouBotMsgToTSim.cpp ${COMMON_INCLUDE_DIR}) 

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/flatToYouBotMsgAddapter/ directory
#
orocos_install_headers( YouBotMsgToTSim.hpp ) # ...you may add multiple header files

