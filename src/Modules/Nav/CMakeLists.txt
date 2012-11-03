
# Files
set(NAV_SOURCES
    Planner.cpp)


# Library
add_library(nav STATIC ${NAV_SOURCES})

include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Nav/")
