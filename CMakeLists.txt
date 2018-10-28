cmake_minimum_required(VERSION 3.0)

project(streetsmarts)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")


find_package(OpenCV REQUIRED)
find_package(OpenGL)
#find_package(PCL)
find_package (Eigen3)
find_package(Open3D)
find_package(realsense2)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

add_definitions(-DUNIX)
add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-unused-result)
add_definitions(-O3)

set(IMGUI /home/ben/librealsense/third-party/imgui/)
aux_source_directory(${IMGUI} IMGUI_SOURCES)
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${OpenCV_INCLUDE_DIRS}
  ${IMGUI}
  ${relasense_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
  /usr/local/Open3d/include/
  /usr/local/cuda/include/
) 

link_directories(${Open3D_LIBRARY_DIRS})

set(DEPENDENCIES ${realsense2_LIBRARY} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} glfw ${Open3D_LIBRARIES} Eigen3::Eigen /media/ssd/Open3D/build/lib/libCuda.a)

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

#set(OPENCV_SOURCES src/opencv.cpp)
#add_executable(opencv ${OPENCV_SOURCES})
#target_link_libraries(opencv ${DEPENDENCIES})

set(OPEN3D_SOURCE src/open3d.cpp src/utils.cpp)
add_executable(open3d ${OPEN3D_SOURCE})
target_link_libraries(open3d ${DEPENDENCIES})

#set(MESH_SOURCES src/mesh.cpp src/utils.cpp)
#add_executable(mesh ${MESH_SOURCES})
#target_link_libraries(mesh ${DEPENDENCIES})

#set(PCL_SOURCES src/pcl.cpp src/utils.cpp)
#add_executable(pcl_convert ${PCL_SOURCES})
#target_link_libraries(pcl_convert ${DEPENDENCIES})
#
#set(PCL_SOURCES src/registration.cpp src/utils.cpp)
#add_executable(register ${PCL_SOURCES})
#target_link_libraries(register ${DEPENDENCIES})

#set(GPU_SOURCES src/gpu_seg.cpp src/utils.cpp)
#add_executable(gpu_seg ${GPU_SOURCES})
#target_link_libraries(gpu_seg ${DEPENDENCIES})
