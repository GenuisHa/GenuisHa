# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ucar_commander: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ucar_commander_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" NAME_WE)
add_custom_target(_ucar_commander_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucar_commander" "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ucar_commander
  "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_commander
)

### Generating Module File
_generate_module_cpp(ucar_commander
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_commander
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ucar_commander_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ucar_commander_generate_messages ucar_commander_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" NAME_WE)
add_dependencies(ucar_commander_generate_messages_cpp _ucar_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_commander_gencpp)
add_dependencies(ucar_commander_gencpp ucar_commander_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_commander_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ucar_commander
  "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_commander
)

### Generating Module File
_generate_module_eus(ucar_commander
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_commander
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ucar_commander_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ucar_commander_generate_messages ucar_commander_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" NAME_WE)
add_dependencies(ucar_commander_generate_messages_eus _ucar_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_commander_geneus)
add_dependencies(ucar_commander_geneus ucar_commander_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_commander_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ucar_commander
  "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_commander
)

### Generating Module File
_generate_module_lisp(ucar_commander
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_commander
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ucar_commander_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ucar_commander_generate_messages ucar_commander_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" NAME_WE)
add_dependencies(ucar_commander_generate_messages_lisp _ucar_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_commander_genlisp)
add_dependencies(ucar_commander_genlisp ucar_commander_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_commander_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ucar_commander
  "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_commander
)

### Generating Module File
_generate_module_nodejs(ucar_commander
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_commander
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ucar_commander_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ucar_commander_generate_messages ucar_commander_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" NAME_WE)
add_dependencies(ucar_commander_generate_messages_nodejs _ucar_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_commander_gennodejs)
add_dependencies(ucar_commander_gennodejs ucar_commander_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_commander_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ucar_commander
  "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_commander
)

### Generating Module File
_generate_module_py(ucar_commander
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_commander
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ucar_commander_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ucar_commander_generate_messages ucar_commander_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/genius/ustb_17th_samrtcar_race/First_race/ustb_17th_smartcar/src/ucar_commander/srv/nav_start.srv" NAME_WE)
add_dependencies(ucar_commander_generate_messages_py _ucar_commander_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_commander_genpy)
add_dependencies(ucar_commander_genpy ucar_commander_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_commander_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_commander
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ucar_commander_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_commander
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ucar_commander_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_commander
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ucar_commander_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_commander)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_commander
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ucar_commander_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_commander)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_commander\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_commander
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ucar_commander_generate_messages_py std_msgs_generate_messages_py)
endif()
