message("***************************************************")
message("*                                                 *")
message("*                 Find libyuv                     *")
message("*                                                 *")
message("***************************************************")

set(YUV_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libyuv/include")
message("include:\n${YUV_INCLUDE_DIR}")

find_library(YUV_LIBRARY NAMES yuv PATHS "${CATKIN_DEVEL_PREFIX}/lib")
message("library:\n${YUV_LIBRARY}")

set(YUV_FOUND TRUE)