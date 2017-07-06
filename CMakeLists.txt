cmake_minimum_required(VERSION 2.8.3)
project(homer_map_manager)
set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp roslib tf homer_mapnav_msgs homer_nav_libs cmake_modules std_srvs interactive_markers)

find_package( Eigen3 REQUIRED )

# eigen 3.2 (wily) only provdies EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(CMAKE_BUILD_TYPE Release)

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

add_library(homerManagers ${Managers_SRC})
target_link_libraries(homerManagers homerImage_io)
add_dependencies(homerManagers ${catkin_EXPORTED_TARGETS})


add_library(homerImage_io
	src/MapIO/image_loader.cpp
	src/MapIO/map_saver.cpp
	src/MapIO/map_loader.cpp
)
target_link_libraries(homerImage_io SDL SDL_image yaml-cpp)
add_dependencies(homerImage_io ${catkin_EXPORTED_TARGETS})

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		roscpp
		roslib
		tf
		homer_mapnav_msgs
		homer_nav_libs
		std_srvs
        interactive_markers
	LIBRARIES homerImage_io homerManagers
)

add_executable(map_manager src/map_manager_node.cpp)

target_link_libraries(
	map_manager
		${catkin_LIBRARIES}
		homerManagers
		homerImage_io
)
add_dependencies( map_manager ${catkin_EXPORTED_TARGETS})

add_executable(poi_visualization src/poi_visualization.cpp)
target_link_libraries(poi_visualization ${catkin_LIBRARIES})
add_dependencies(poi_visualization ${catkin_EXPORTED_TARGETS})


install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS homerManagers homerImage_io map_manager poi_visualization
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
