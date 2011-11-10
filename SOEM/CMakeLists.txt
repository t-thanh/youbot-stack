cmake_minimum_required(VERSION 2.4.6)

SET(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
#SET(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g")

link_libraries( pthread)

include_directories(
  src
)

ADD_LIBRARY(soem STATIC
  src/ethercatbase.c 
  src/ethercatcoe.c 
  src/ethercatconfig.c 
  src/ethercatfoe.c 
  src/ethercatsoe.c 
  src/ethercatmain.c 
  src/ethercatprint.c 
  src/nicdrv.c 
  src/ethercatdc.c
) 

ADD_LIBRARY(soemShared SHARED
  src/ethercatbase.c 
  src/ethercatcoe.c 
  src/ethercatconfig.c 
  src/ethercatfoe.c 
  src/ethercatsoe.c 
  src/ethercatmain.c 
  src/ethercatprint.c 
  src/nicdrv.c 
  src/ethercatdc.c
)
SET_TARGET_PROPERTIES(soemShared PROPERTIES OUTPUT_NAME "soem")


SET(SOEM_HEADERS
  src/ethercatbase.h
  src/ethercatcoe.h
  src/ethercatconfig.h
  src/ethercatconfiglist.h
  src/ethercatdc.h
  src/ethercatfoe.h 
  src/ethercatmain.h 
  src/ethercatprint.h 
  src/ethercatsoe.h 
  src/ethercattype.h
  src/nicdrv.h 
) 

MACRO(INSTALL_HEADERS_WITH_DIRECTORY HEADER_LIST)
  FOREACH(HEADER ${${HEADER_LIST}})
    STRING(REGEX MATCH "(.*)[/\\]" DIR ${HEADER})
    INSTALL(FILES ${HEADER} DESTINATION include/soem/${DIR})
  ENDFOREACH(HEADER)
ENDMACRO(INSTALL_HEADERS_WITH_DIRECTORY)

INSTALL_HEADERS_WITH_DIRECTORY(SOEM_HEADERS)

INSTALL(TARGETS soem soemShared LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

