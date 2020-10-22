PROJECT(nns_ros2_bridge C CXX)

IF(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
ENDIF()

FIND_PROGRAM(PYTHON_EXECUTABLE python3 DOC "python interpreter")
IF(PYTHON_EXECUTABLE)
  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} -c "import platform; ver = platform.python_version_tuple(); print(ver[0] + '.' + ver[1])"
                  OUTPUT_VARIABLE PYTHON_VER
                  RESULT_VARIABLE PYTHON_VER_NOT_FOUND
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDIF(PYTHON_EXECUTABLE)

IF(PYTHON_VER_NOT_FOUND)
  MESSAGE(FATAL_ERROR "Failed to find python3 required by ament_cmake")
ENDIF(PYTHON_VER_NOT_FOUND)

IF(EXISTS ${CMAKE_PREFIX_PATH}/lib/python${PYTHON_VER}/site-packages)
  LIST(APPEND PYTHONPATH_CANDIDATES "${CMAKE_PREFIX_PATH}/lib/python${PYTHON_VER}")
ENDIF(EXISTS ${CMAKE_PREFIX_PATH}/lib/python${PYTHON_VER}/site-packages)

IF(EXISTS ${CMAKE_PREFIX_PATH}/lib/python3/site-packages)
  LIST(APPEND PYTHONPATH_CANDIDATES "${CMAKE_PREFIX_PATH}/lib/python3")
ENDIF(EXISTS ${CMAKE_PREFIX_PATH}/lib/python3/site-packages)

LIST(LENGTH PYTHONPATH_CANDIDATES LEN_PYTHONPATH_CANDIDATES)
IF(${LEN_PYTHONPATH_CANDIDATES} LESS "1")
  MESSAGE(FATAL_ERROR "Failed to find python3-related path in ${CMAKE_PREFIX_PATH}/lib")
ENDIF(${LEN_PYTHONPATH_CANDIDATES} LESS "1")

FOREACH(pythonpath ${PYTHONPATH_CANDIDATES})
  SET(ENV{PYTHONPATH} ${pythonpath}/site-packages:$ENV{PYTHONPATH})
ENDFOREACH()

# find dependencies
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)

FIND_PACKAGE(rosidl_default_generators REQUIRED)

SET(NNS_ROS2_LIB_MSGS "${NNS_ROS_COMMON_DIR}/msg:Tensors.msg")
SET(NNS_ROS2_LIB_SRCS
    node/nns_rclcpp_publisher.cc
)
SET(NNS_ROS2_LIB_TARGET nns_ros_bridge2)

ROSIDL_GENERATE_INTERFACES(${PROJECT_NAME}
    ${NNS_ROS2_LIB_MSGS}
    DEPENDENCIES builtin_interfaces std_msgs
)

ADD_LIBRARY(${NNS_ROS2_LIB_TARGET} SHARED
    ${NNS_ROS2_LIB_SRCS}
)
AMENT_TARGET_DEPENDENCIES(${NNS_ROS2_LIB_TARGET} rclcpp std_msgs)
AMENT_EXPORT_DEPENDENCIES(rosidl_default_runtime)
ROSIDL_TARGET_INTERFACES(${NNS_ROS2_LIB_TARGET}
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

IF(BUILD_TESTING)
  FIND_PACKAGE(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  AMENT_LINT_AUTO_FIND_TEST_DEPENDENCIES()
ENDIF()

AMENT_PACKAGE()
INSTALL(TARGETS ${NNS_ROS2_LIB_TARGET}
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
)
