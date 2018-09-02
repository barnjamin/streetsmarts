cmake_minimum_required(VERSION 3.0)

project(streetsmarts)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")


find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(PCL)

set(IMGUI /home/ben/librealsense/third-party/imgui/)
aux_source_directory(${IMGUI} IMGUI_SOURCES)
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${IMGUI}
) 

set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} glfw)

set(APP_SOURCES src/region_growing.cpp)
add_executable(region_growing ${APP_SOURCES})
target_link_libraries(region_growing ${DEPENDENCIES})

set(APP_SOURCES src/seg.cpp)
add_executable(seg ${APP_SOURCES})
target_link_libraries(seg ${DEPENDENCIES})

set(MESH_SOURCES src/mesh.cpp)
add_executable(mesh ${MESH_SOURCES})
target_link_libraries(mesh ${DEPENDENCIES})

set(NORMAL_SOURCES src/pcl.cpp)
add_executable(normal ${NORMAL_SOURCES})
target_link_libraries(normal ${DEPENDENCIES})


# shared library
#set(LIB_SOURCES src/street.cpp)
#set(LIBRARY_NAME World)
#add_library(${LIBRARY_NAME} SHARED ${LIB_SOURCES})
#target_link_libraries(${PROJECT_NAME} ${LIBRARY_NAME})
