# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tictoc_profiler: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itictoc_profiler:/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tictoc_profiler_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" NAME_WE)
add_custom_target(_tictoc_profiler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tictoc_profiler" "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tictoc_profiler
  "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tictoc_profiler
)

### Generating Services

### Generating Module File
_generate_module_cpp(tictoc_profiler
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tictoc_profiler
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tictoc_profiler_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tictoc_profiler_generate_messages tictoc_profiler_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" NAME_WE)
add_dependencies(tictoc_profiler_generate_messages_cpp _tictoc_profiler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tictoc_profiler_gencpp)
add_dependencies(tictoc_profiler_gencpp tictoc_profiler_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tictoc_profiler_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tictoc_profiler
  "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tictoc_profiler
)

### Generating Services

### Generating Module File
_generate_module_eus(tictoc_profiler
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tictoc_profiler
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tictoc_profiler_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tictoc_profiler_generate_messages tictoc_profiler_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" NAME_WE)
add_dependencies(tictoc_profiler_generate_messages_eus _tictoc_profiler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tictoc_profiler_geneus)
add_dependencies(tictoc_profiler_geneus tictoc_profiler_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tictoc_profiler_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tictoc_profiler
  "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tictoc_profiler
)

### Generating Services

### Generating Module File
_generate_module_lisp(tictoc_profiler
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tictoc_profiler
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tictoc_profiler_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tictoc_profiler_generate_messages tictoc_profiler_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" NAME_WE)
add_dependencies(tictoc_profiler_generate_messages_lisp _tictoc_profiler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tictoc_profiler_genlisp)
add_dependencies(tictoc_profiler_genlisp tictoc_profiler_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tictoc_profiler_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tictoc_profiler
  "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tictoc_profiler
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tictoc_profiler
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tictoc_profiler
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tictoc_profiler_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tictoc_profiler_generate_messages tictoc_profiler_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" NAME_WE)
add_dependencies(tictoc_profiler_generate_messages_nodejs _tictoc_profiler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tictoc_profiler_gennodejs)
add_dependencies(tictoc_profiler_gennodejs tictoc_profiler_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tictoc_profiler_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tictoc_profiler
  "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tictoc_profiler
)

### Generating Services

### Generating Module File
_generate_module_py(tictoc_profiler
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tictoc_profiler
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tictoc_profiler_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tictoc_profiler_generate_messages tictoc_profiler_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/benchun/dataset/icl_nuim/orb_cuboid_plane_slam_V4/Thirdparty/tictoc_profiler/msg/ProfilerEntry.msg" NAME_WE)
add_dependencies(tictoc_profiler_generate_messages_py _tictoc_profiler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tictoc_profiler_genpy)
add_dependencies(tictoc_profiler_genpy tictoc_profiler_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tictoc_profiler_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tictoc_profiler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tictoc_profiler
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tictoc_profiler_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tictoc_profiler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tictoc_profiler
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tictoc_profiler_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tictoc_profiler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tictoc_profiler
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tictoc_profiler_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tictoc_profiler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tictoc_profiler
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tictoc_profiler_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tictoc_profiler)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tictoc_profiler\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tictoc_profiler
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tictoc_profiler_generate_messages_py std_msgs_generate_messages_py)
endif()
