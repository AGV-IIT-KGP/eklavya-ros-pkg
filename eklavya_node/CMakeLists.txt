cmake_minimum_required(VERSION 2.6)
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

find_package(MRPT REQUIRED base slam hwdrivers)# WARNING: Add all the MRPT libs used by your program: "gui", "obs", "slam",etc. 
LINK_DIRECTORIES(${MRPT_DIR}/lib)
LINK_DIRECTORIES(${MRPT_DIR}/lib/otherlibs)
LINK_DIRECTORIES(${MRPT_SOURCE_DIR}/lib/otherlibs)
LINK_DIRECTORIES("/usr/local/lib/")

#Options
option (USE_IMU "Use IMU" ON) 
option (USE_LIDAR "Use Lidar" ON) 
option (USE_GPS "Use GPS" ON)
option (USE_ENCODER "Use Encoder" ON)
option (USE_EKF "Use EKF" ON)
option (USE_SLAM "Use SLAM" ON)
option (USE_NAVIGATION "Use Navigation" ON)
option (USE_PLANNER "Use Planner" ON)
option (USE_SERIAL_PORT "Use SerialPortLinux" ON) 
option (USE_DIAGNOSTICS "Use Diagnostics" ON)

include_directories ("${PROJECT_SOURCE_DIR}/src/")

if (USE_IMU)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/IMU/")
rosbuild_add_library(IMULib src/Modules/IMU/IMU.cpp src/Modules/IMU/imu_thread.cpp)
endif ()

if (USE_LIDAR)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Lidar")
rosbuild_add_library(LidarLib src/Modules/Lidar/LidarData.cpp src/Modules/Lidar/lidar_thread.cpp)
endif ()

if (USE_GPS)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/GPS/")
#rosbuild_add_library(GPSLib src/Modules/GPS/GPS.cpp src/Modules/GPS/gps_thread.cpp)
rosbuild_add_library(GPSLib src/Modules/GPS/gps.cpp src/Modules/GPS/gps_thread.cpp)
endif ()

if (USE_ENCODER)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Encoder/")
rosbuild_add_library(EncoderLib src/Modules/Encoder/encoder_thread.cpp)
endif ()

if (USE_EKF)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/EKF/")
rosbuild_add_library(EKFLib src/Modules/EKF/ekf_thread.cpp)
endif ()

if (USE_SLAM)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/SLAM/")
rosbuild_add_library(SLAMLib src/Modules/SLAM/slam.cpp src/Modules/SLAM/slam_thread.cpp)
endif ()

if (USE_NAVIGATION)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Navigation/")
rosbuild_add_library(NavigationLib src/Modules/Navigation/navigation.cpp                                    src/Modules/Navigation/follow_nose_strategy.cpp src/Modules/Navigation/track_way_point_strategy.cpp                                    src/Modules/Navigation/navigation_thread.cpp)
endif ()

if (USE_PLANNER)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Planner/")
#rosbuild_add_library(PlannerLib src/Modules/Planner/Planner.cpp src/Modules/Planner/planner_thread.cpp)
rosbuild_add_library(PlannerLib src/Modules/Planner/planner.cpp src/Modules/Planner/planner_thread.cpp)
endif ()

if (USE_DIAGNOSTICS)
include_directories ("${PROJECT_SOURCE_DIR}/src/Modules/Diagnostics/")
rosbuild_add_library(DiagnosticsLib src/Modules/Diagnostics/diagnostics.cpp src/Modules/Diagnostics/diagnostics_thread.cpp)
endif ()

if (USE_SERIAL_PORT)
include_directories ("${PROJECT_SOURCE_DIR}/src/Utils/SerialPortLinux/")
rosbuild_add_library(SerialPortLinuxLib src/Utils/SerialPortLinux/serial_lnx.cpp)
endif ()

##cvBlob
include_directories ("${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/")
set(cvBlob_CVBLOB  ${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/cvblob.cpp
                   ${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/cvlabel.cpp
		   ${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/cvaux.cpp
		   ${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/cvcontour.cpp
		   ${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/cvtrack.cpp
		   ${PROJECT_SOURCE_DIR}/src/ExternalLib/cvBlob/cvcolor.cpp
)

set_source_files_properties(${cvBlob_SRC}
                            PROPERTIES
                            COMPILE_FLAGS "-O3"
)
rosbuild_add_library(cvblob SHARED ${cvBlob_CVBLOB})
target_link_libraries(cvblob ${OpenCV_LIBS})
##cvBlob end




rosbuild_add_executable(${PROJECT_NAME} src/eklavya2.cpp)
target_link_libraries (${PROJECT_NAME} IMULib LidarLib GPSLib EncoderLib EKFLib SLAMLib PlannerLib NavigationLib DiagnosticsLib SerialPortLinuxLib)

#target_link_libraries (${PROJECT_NAME}  ${EXTRA_LIBS})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries( ${PROJECT_NAME} ${MRPT_LIBS} )
target_link_libraries( ${PROJECT_NAME} cvblob)
