cmake_minimum_required(VERSION 3.16)
project( NTSD-VisualOdometry )
find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_subdirectory(external)

include_directories( ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${V3D_INCLUDE_DIRS} include)
file(GLOB_RECURSE SOURCE_FILES 
	${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} V3D)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17 )