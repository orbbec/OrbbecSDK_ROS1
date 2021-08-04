message("***************************************************")
message("*                                                 *")
message("*                 Find Orbbec SDK                 *")
message("*                                                 *")
message("***************************************************")

set(ORBBEC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/orbbec")

find_path(ORBBEC_SDK_INCLUDE_DIR "libobsensor/ObSensor.hpp" "${ORBBEC_SDK_DIR}/include")
message("include:\n${ORBBEC_SDK_INCLUDE_DIR}")

find_library(ORBBEC_SDK_LIBRARY NAMES OrbbecSDK PATHS "${CMAKE_CURRENT_SOURCE_DIR}/orbbec/lib")
find_library(D2C_LIBRARY NAMES d2c PATHS "${CMAKE_CURRENT_SOURCE_DIR}/orbbec/lib")
find_library(POSTFILTER_LIBRARY NAMES postfilter PATHS "${CMAKE_CURRENT_SOURCE_DIR}/orbbec/lib")
set(ORBBEC_SDK_LIBRARIES ${ORBBEC_SDK_LIBRARY} ${D2C_LIBRARY} ${POSTFILTER_LIBRARY})
message("libraries:\n${ORBBEC_SDK_LIBRARIES}")

set(ORBBEC_SDK_FOUND TRUE)