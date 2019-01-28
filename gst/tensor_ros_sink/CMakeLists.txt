ADD_LIBRARY(tensor_ros_sink tensor_ros_sink.c)
ADD_DEPENDENCIES(tensor_ros_sink nns_ros_bridge)
TARGET_LINK_LIBRARIES(tensor_ros_sink
    nns_ros_bridge
    ${PKGS_COMMON_LIBRARIES}
    )

INSTALL(TARGETS tensor_ros_sink
	RUNTIME DESTINATION ${EXEC_PREFIX}
	LIBRARY DESTINATION ${GST_INSTALL_DIR}
	ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
)
