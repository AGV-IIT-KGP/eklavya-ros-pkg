cmake_minimum_required (VERSION 2.6)
PROJECT(Eklavya)
# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)
# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
# and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#Options
option (USE_LANE  "Use tutorial provided math implementation" ON) 

# add the LANE library?
#
#if (USE_LANE)
#  include_directories ("${PROJECT_SOURCE_DIR}/Modules/Lane")
#  add_subdirectory (Lane) 
#  set (EXTRA_LIBS ${EXTRA_LIBS} Lane)
#message(STATUS "EXTRA_LIBS: ${EXTRA_LIBS}")
#endif (USE_LANE)

find_package( OpenCV REQUIRED )
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
find_package(MRPT REQUIRED base slam hwdrivers)# WARNING: Add all the MRPT libs used by your program: "gui", "obs", "slam",etc. 

include_directories ("${PROJECT_SOURCE_DIR}/Modules/Nav/")
include_directories ("${PROJECT_SOURCE_DIR}/Modules/IMU/")
include_directories ("${PROJECT_SOURCE_DIR}/Modules/GPS/")
include_directories ("${PROJECT_SOURCE_DIR}/Modules/Lane/")
include_directories ("${PROJECT_SOURCE_DIR}/Modules/Lidar")
include_directories ("${PROJECT_SOURCE_DIR}/Utils/SerialPortLinux/")

#FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
#INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

LINK_DIRECTORIES(${MRPT_DIR}/lib)
LINK_DIRECTORIES(${MRPT_DIR}/lib/otherlibs)
LINK_DIRECTORIES(${MRPT_SOURCE_DIR}/lib/otherlibs)


#find_package(PCL 1.5 REQUIRED)
#add_definitions(${PCL_DEFINITIONS})


#add_subdirectory (MathFunctions) 

#add_executable(eklavya AGV.cpp)
add_executable(eklavya AGV.cpp ./Modules/IMU/IMU.cpp ./Modules/Nav/Planner.cpp ./Utils/SerialPortLinux/serial_lnx.cpp ./Modules/Lidar/LidarData.cpp ./Modules/GPS/GPS.cpp ./Modules/Lane/Lane.cpp)

#target_link_libraries (eklavya Lane)
target_link_libraries (eklavya  ${EXTRA_LIBS})
target_link_libraries( eklavya ${OpenCV_LIBS} )
target_link_libraries( eklavya ${MRPT_LIBS} )

# Set optimized building:
#IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
#ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")

