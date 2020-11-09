ADD_EXECUTABLE(unittest_tensor_ros_src unittest_tensor_ros_src.cpp ${GTEST_SRC})
TARGET_INCLUDE_DIRECTORIES(unittest_tensor_ros_src PRIVATE
  ${PKGS_COMMON_INCLUDE_DIRS}
  ${PKGS_TEST_INCLUDE_DIRS}
  ${GTEST_INC}
)

TARGET_LINK_LIBRARIES(unittest_tensor_ros_src
  ${PKGS_COMMON_LIBRARIES}
  ${PKGS_TEST_LIBRARIES}
  ${GTEST_LINK_LIBS}
)

ADD_TEST(NAME unittest_tensor_ros_src.properties
  COMMAND unittest_tensor_ros_src --gtest_filter=test_tensor_ros_src.properties)
