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
 /usr/local/cuda/include/
) 

set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} glfw)

#set(APP_SOURCES src/don_seg.cpp src/utils.cpp)
#add_executable(don_seg ${APP_SOURCES})
#target_link_libraries(don_seg ${DEPENDENCIES})
#
#set(APP_SOURCES src/region_growing.cpp src/utils.cpp)
#add_executable(region_growing ${APP_SOURCES})
#target_link_libraries(region_growing ${DEPENDENCIES})
#
#set(APP_SOURCES src/seg.cpp src/utils.cpp)
#add_executable(seg ${APP_SOURCES})
#target_link_libraries(seg ${DEPENDENCIES})
#
set(MESH_SOURCES src/mesh.cpp src/utils.cpp)
add_executable(mesh ${MESH_SOURCES})
target_link_libraries(mesh ${DEPENDENCIES})
#
#set(NORMAL_SOURCES src/pcl.cpp src/utils.cpp)
#add_executable(normal ${NORMAL_SOURCES})
#target_link_libraries(normal ${DEPENDENCIES})
#
#set(PCL_SOURCES src/registration.cpp src/utils.cpp)
#add_executable(register ${PCL_SOURCES})
#target_link_libraries(register ${DEPENDENCIES})

#set(GPU_SOURCES src/gpu_seg.cpp src/utils.cpp)
#add_executable(gpu_seg ${GPU_SOURCES})
#target_link_libraries(gpu_seg ${DEPENDENCIES})
