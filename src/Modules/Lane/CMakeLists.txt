
# Files
set(LANE_SOURCES
    Lane.cpp)


# Library
add_library(lane STATIC ${LANE_SOURCES})

include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Lane/")
