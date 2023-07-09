cmake_minimum_required(VERSION 3.15)
project (wfc VERSION 0.1.0)

add_library (wfc source/wfc.cpp)
target_include_directories (wfc PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include/wfc>
)
set_target_properties (wfc PROPERTIES CXX_STANDARD 17)


add_executable (test_wfc main.cpp)
set_target_properties (test_wfc PROPERTIES CXX_STANDARD 17)
target_link_libraries (test_wfc PRIVATE wfc)

configure_file (config.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake @ONLY)
configure_file (config-version.cmake.in ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake @ONLY)

install(FILES include/wfc.hpp DESTINATION "include/wfc")

install (TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets
	ARCHIVE DESTINATION "lib/${PROJECT_NAME}-${PROJECT_VERSION}" 
)
install (FILES
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
	${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
	DESTINATION "lib/${PROJECT_NAME}-${PROJECT_VERSION}"
)

install (EXPORT ${PROJECT_NAME}-targets
	DESTINATION "lib/${PROJECT_NAME}-${PROJECT_VERSION}" 
)
