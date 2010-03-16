SET(module_name ${PROJECT_NAME})
add_library(${PROJECT_NAME} SHARED CamFireWire.cpp filter/frame2rggb.cpp)
target_link_libraries(${PROJECT_NAME} rt base dc1394 camera_firewire camera_interface)

install(TARGETS ${PROJECT_NAME} 
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
    )

SET(TestViewer TestViewer)
SET(module_name ${TestViewer})
include_directories(/usr/local/include/opencv)
add_executable(${TestViewer} TestViewer.cpp)   
target_link_libraries(${TestViewer} cv highgui camera_firewire)
install(TARGETS ${TestViewer} RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/test)

