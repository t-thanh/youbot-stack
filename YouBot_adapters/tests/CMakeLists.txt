INCLUDE_DIRECTORIES(${COMMON_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

rosbuild_add_executable(H2TF_test H2TF_test.cpp)
orocos_component(H2TF_oro_test H2TF_oro_test.cpp)
