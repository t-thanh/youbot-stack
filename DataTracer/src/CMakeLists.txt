INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR} ${SOEM_INCLUDE_DIR} ${OODL_YOUBOT_INCLUDE_DIR})

SET(libDataTracer_libs
	boost_thread
)

LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

orocos_component(Sink Sink.hpp Sink.cpp DTTypes.hpp DTFactory.hpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(Sink ${libDataTracer_libs})

orocos_component(CSVFileWriter CSVFileWriter.hpp CSVFileWriter.cpp ${COMMON_INCLUDE_DIR})

TARGET_LINK_LIBRARIES(CSVFileWriter ${libDataTracer_libs})