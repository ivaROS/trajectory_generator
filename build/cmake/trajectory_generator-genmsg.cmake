# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "trajectory_generator: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itrajectory_generator:/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(trajectory_generator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg" NAME_WE)
add_custom_target(_trajectory_generator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_generator" "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg" "trajectory_generator/trajectory_point:std_msgs/Header"
)

get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg" NAME_WE)
add_custom_target(_trajectory_generator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "trajectory_generator" "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(trajectory_generator
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg"
  "${MSG_I_FLAGS}"
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_generator
)
_generate_msg_cpp(trajectory_generator
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_generator
)

### Generating Services

### Generating Module File
_generate_module_cpp(trajectory_generator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_generator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(trajectory_generator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(trajectory_generator_generate_messages trajectory_generator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg" NAME_WE)
add_dependencies(trajectory_generator_generate_messages_cpp _trajectory_generator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg" NAME_WE)
add_dependencies(trajectory_generator_generate_messages_cpp _trajectory_generator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_generator_gencpp)
add_dependencies(trajectory_generator_gencpp trajectory_generator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_generator_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(trajectory_generator
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg"
  "${MSG_I_FLAGS}"
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_generator
)
_generate_msg_lisp(trajectory_generator
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_generator
)

### Generating Services

### Generating Module File
_generate_module_lisp(trajectory_generator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_generator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(trajectory_generator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(trajectory_generator_generate_messages trajectory_generator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg" NAME_WE)
add_dependencies(trajectory_generator_generate_messages_lisp _trajectory_generator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg" NAME_WE)
add_dependencies(trajectory_generator_generate_messages_lisp _trajectory_generator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_generator_genlisp)
add_dependencies(trajectory_generator_genlisp trajectory_generator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_generator_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(trajectory_generator
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg"
  "${MSG_I_FLAGS}"
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_generator
)
_generate_msg_py(trajectory_generator
  "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_generator
)

### Generating Services

### Generating Module File
_generate_module_py(trajectory_generator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_generator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(trajectory_generator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(trajectory_generator_generate_messages trajectory_generator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_points.msg" NAME_WE)
add_dependencies(trajectory_generator_generate_messages_py _trajectory_generator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yipuzhao/ros_workspace/package_dir/trajectory_generator/msg/trajectory_point.msg" NAME_WE)
add_dependencies(trajectory_generator_generate_messages_py _trajectory_generator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(trajectory_generator_genpy)
add_dependencies(trajectory_generator_genpy trajectory_generator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS trajectory_generator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_generator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/trajectory_generator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(trajectory_generator_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_generator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/trajectory_generator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(trajectory_generator_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_generator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_generator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/trajectory_generator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(trajectory_generator_generate_messages_py std_msgs_generate_messages_py)
