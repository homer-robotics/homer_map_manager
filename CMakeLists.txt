cmake_minimum_required(VERSION 2.8.3)
project(homer_map_manager)

find_package(catkin REQUIRED COMPONENTS roscpp roslib tf homer_mapnav_msgs homer_nav_libs cmake_modules homer_tools)

find_package( Eigen3 REQUIRED )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

set(Managers_SRC
	src/Managers/MapManager.cpp
	src/Managers/MaskingManager.cpp
	src/Managers/PoiManager.cpp
	src/Managers/RoiManager.cpp
)

add_library(Managers ${Managers_SRC})
target_link_libraries(Managers image_io)
add_dependencies(Managers homer_mapnav_msgs_gencpp)


add_library(image_io
	src/MapIO/image_loader.cpp
	src/MapIO/map_saver.cpp
	src/MapIO/map_loader.cpp
)
target_link_libraries(image_io SDL SDL_image yaml-cpp)
add_dependencies(image_io homer_mapnav_msgs_gencpp)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		roscpp
)

add_executable(map_manager src/map_manager_node.cpp)

target_link_libraries(
	map_manager
		${catkin_LIBRARIES}
		Managers
		image_io
)

add_dependencies(
	map_manager
		${catkin_EXPORTED_TARGETS}
)
