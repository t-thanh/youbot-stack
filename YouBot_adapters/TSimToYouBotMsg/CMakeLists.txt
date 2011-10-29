INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libflatToYouBotMsgAddapter-<target>.so
# and installs in the directory lib/orocos/flatToYouBotMsgAddapter/
#
orocos_component(TSimToYouBotMsgAdapter TSimToYouBotMsg.hpp TSimToYouBotMsg.cpp ${COMMON_INCLUDE_DIR}) 
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/flatToYouBotMsgAddapter/ directory
#
orocos_install_headers( TSimToYouBotMsg.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
