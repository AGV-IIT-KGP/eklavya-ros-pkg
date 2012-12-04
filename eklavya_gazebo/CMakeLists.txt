cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_find_ros_package(ogre_tools)
rosbuild_find_ros_package(xacro)
rosbuild_find_ros_package(ivcon)

# iterate through all the obj files
file(GLOB mesh_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.obj)
set(generated_mesh_files "")
set(ogre_mesh_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models)

foreach(it ${mesh_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME)

  IF ( ${basename} MATCHES "_convex" )
    message("ignoring stale .._convex.* file:",${basename})
  ELSE ( ${basename} MATCHES "_convex" )

    #convex directory
    add_custom_command(
      OUTPUT ${basepath}/convex
      COMMAND mkdir -p
      ARGS ${basepath}/convex)
    set(generated_mesh_files ${generated_mesh_files} ${basepath}/convex)

    #create stl files from obj files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}.stlb
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${it} ${basepath}/convex/${basename}.stlb
      DEPENDS ${it} ${basepath}/convex)
    set(generated_mesh_files ${generated_mesh_files} ${basepath}/convex/${basename}.stlb)

    # convert to ogre files
    add_custom_command(
      OUTPUT ${ogre_mesh_out_path}/${basename}.mesh
      COMMAND rosrun 
      ARGS ogre_tools stl_to_mesh ${basepath}/convex/${basename}.stlb ${ogre_mesh_out_path}/${basename}.mesh
      DEPENDS ${basepath}/convex ${basepath}/convex/${basename}.stlb)
    
    set(generated_mesh_files ${generated_mesh_files} ${ogre_mesh_out_path}/${basename}.mesh)

  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)


#update targets
add_custom_target(media_files ALL DEPENDS ${generated_mesh_files})

