
add_definitions(-DBASE_LOG_NAMESPACE=$PROJECT_NAME)

add_library(${PROJECT_NAME} SHARED CamFireWire.cpp filter/frame2rggb.cpp)
target_link_libraries(${PROJECT_NAME} rt ${DC1394_LIBRARIES}
    ${CAM_INTERFACE_LIBRARIES} ${BASE_LIB_LIBRARIES} base-logging)

install(TARGETS ${PROJECT_NAME} 
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
    )

if (TEST_ENABLED)
add_executable(TestViewer TestViewer.cpp)   
target_link_libraries(TestViewer ${OPENCV_LIBRARIES} ${DC1394_LIBRARIES}
    camera_firewire)
endif()

