PROJECT(nns_ros_bridge C CXX)

## Inner functions
FUNCTION(search_pkg_conf_dir_r_inner search_path return_list)
  FILE(GLOB_RECURSE new_list ${search_path}/*.pc)
  SET(dir_list "")
  FOREACH(file_path ${new_list})
    GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
    SET(dir_list ${dir_list} ${dir_path})
  ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list} PARENT_SCOPE)
ENDFUNCTION(search_pkg_conf_dir_r_inner)

FUNCTION(search_pkg_conf_dirs search_paths return_list)
  FOREACH(search_path ${search_paths})
    search_pkg_conf_dir_r_inner(${search_path} inner_return)
    SET(all_list ${all_list} ${inner_return})
  ENDFOREACH()
  SET(${return_list} ${all_list} PARENT_SCOPE)
ENDFUNCTION(search_pkg_conf_dirs)


search_pkg_conf_dirs (${CMAKE_PREFIX_PATH} pkg_config_path_list)
FOREACH(each_path ${pkg_config_path_list})
  SET(ENV{PKG_CONFIG_PATH} "${each_path}:$ENV{PKG_CONFIG_PATH}")
ENDFOREACH()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  genmsg
)

################################################
## Declare ROS messages, services and actions ##
################################################

ADD_MESSAGE_FILES(
  NOINSTALL
  DIRECTORY ${NNS_ROS_COMMON_DIR}/msg
  FILES Tensors.msg
)

## Generate added messages and services with any dependencies listed here
GENERATE_MESSAGES(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
CATKIN_PACKAGE(
  LIBRARIES nns_ros_bridge
  CATKIN_DEPENDS roscpp rosbag std_msgs message_runtime
  DEPENDS ${PKGS_COMMON}
)

SET(CXX_SRC_FILES_NNS_ROS_PUBLISHER
  ./node/nns_roscpp_publisher.cc
)

SET(CXX_SRC_FILES_NNS_ROS_SRC
  ./node/nns_roscpp_subscriber.cc
)

###########
## Build ##
###########

INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)

ADD_LIBRARY(${PROJECT_NAME} ${CXX_SRC_FILES_NNS_ROS_PUBLISHER} ${CXX_SRC_FILES_NNS_ROS_SRC})
ADD_DEPENDENCIES(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PKGS_COMMON_LIBRARIES}
)

IF(CMAKE_HOST_UNIX)
  CATKIN_ADD_ENV_HOOKS(20.gst_plugin_path SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
ENDIF()

#############
## Install ##
#############

INSTALL(
  DIRECTORY ${NNS_ROS_COMMON_DIR}/msg
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.msg"
)

INSTALL(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
)
