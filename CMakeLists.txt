cmake_minimum_required(VERSION 3.16)
project( NTSD-VisualOdometry )
find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories( ${OpenCV_INCLUDE_DIRS} include)
file(GLOB_RECURSE SOURCE_FILES 
	${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17 )