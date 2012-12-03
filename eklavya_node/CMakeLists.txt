cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
# and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV REQUIRED)
#LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

#Options
option (USE_LANE  "Use LANE" ON) 
option (USE_NAV  "Use Nav" ON) 
option (USE_IMU  "Use IMU" ON) 
option (USE_GPS  "Use GPS" ON) 
option (USE_LIDAR  "Use Lidar" ON) 
option (USE_SERIAL_PORT  "Use SerialPortLinux" ON) 

# add the LANE library?
if (USE_LANE)
 # include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Lane")
  add_subdirectory ("${PROJECT_SOURCE_DIR}/src/Modules/Lane") 
  #set (EXTRA_LIBS ${EXTRA_LIBS} lane)
  #message(STATUS "EXTRA_LIBS: ${EXTRA_LIBS}")
endif (USE_LANE)

if (USE_NAV)
  add_subdirectory ("${PROJECT_SOURCE_DIR}/src/Modules/Nav") 
endif (USE_NAV)


include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/IMU/")
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/GPS/")

include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Lidar")
include_directories ("${PROJECT_SOURCE_DIR}/src/Utils/SerialPortLinux/")

find_package(MRPT REQUIRED base slam hwdrivers)# WARNING: Add all the MRPT libs used by your program: "gui", "obs", "slam",etc. 
LINK_DIRECTORIES(${MRPT_DIR}/lib)
LINK_DIRECTORIES(${MRPT_DIR}/lib/otherlibs)
LINK_DIRECTORIES(${MRPT_SOURCE_DIR}/lib/otherlibs)



rosbuild_add_executable(${PROJECT_NAME} src/AGV.cpp ./src/Modules/IMU/IMU.cpp  ./src/Utils/SerialPortLinux/serial_lnx.cpp ./src/Modules/Lidar/LidarData.cpp ./src/Modules/GPS/GPS.cpp )


#rosbuild_add_executable(${PROJECT_NAME} src/AGV.cpp )


#target_link_libraries (${PROJECT_NAME}  ${EXTRA_LIBS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries( ${PROJECT_NAME} ${MRPT_LIBS} )
