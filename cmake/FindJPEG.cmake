message("***************************************************")
message("*                                                 *")
message("*                 Find libjpeg                    *")
message("*                                                 *")
message("***************************************************")

set(JPEG_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libjpeg")
message("include:\n${JPEG_INCLUDE_DIR}")

find_library(JPEG_LIBRARY NAMES jpeg PATHS "${CATKIN_DEVEL_PREFIX}/lib")
message("library:\n${JPEG_LIBRARY}")

set(JPEG_FOUND TRUE)