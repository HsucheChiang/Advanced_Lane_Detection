cmake_minimum_required(VERSION 3.5.1)
project(laneDetection)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
add_subdirectory(src)
set(CMAKE_INSTALL_PREFIX ..)

#Files installation path
install(DIRECTORY camera_cal/ 
	DESTINATION LaneDetection/camera_cal)
install(DIRECTORY build/bin/
	DESTINATION LaneDetection/
	FILES_MATCHING PATTERN laneDetection
	PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                    GROUP_EXECUTE GROUP_READ)
install(DIRECTORY video/
	DESTINATION LaneDetection/)
