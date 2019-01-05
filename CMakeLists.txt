cmake_minimum_required(VERSION 3.0)

project(streetsmarts)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")


find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(GLUT)
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
  ${OPENGL_INCLUDE_DIRS}  
  ${GLUT_INCLUDE_DIRS}
) 

link_directories(${Open3D_LIBRARY_DIRS})

set(DEPENDENCIES 
	${GLUT_LIBRARY}
	${realsense2_LIBRARY} 
	${OPENGL_LIBRARIES} 
	${OpenCV_LIBS} 
	glfw 
	${Open3D_LIBRARIES} 
	Eigen3::Eigen 
	/home/ben/Open3D/build/lib/libCuda.a
)


#set(CUDA_ALIGN_SOURCES src/pcl-cuda.cpp src/utils.cpp)
#add_executable(pcl-cuda ${CUDA_ALIGN_SOURCES})
#target_link_libraries(pcl-cuda ${DEPENDENCIES})


#set(OPEN3D_SOURCE src/open3d.cpp src/utils.cpp src/MadgwickAHRS.c)
#add_executable(open3d ${OPEN3D_SOURCE})
#target_link_libraries(open3d ${DEPENDENCIES})

#set(DUMP_SOURCE src/dump.cpp src/utils.cpp src/MadgwickAHRS.c)
#add_executable(dump ${DUMP_SOURCE})
#target_link_libraries(dump ${DEPENDENCIES})

set(LOAD_SOURCE src/load.cpp src/utils.cpp src/pose.cpp src/display.cpp)
add_executable(load ${LOAD_SOURCE})
target_link_libraries(load ${DEPENDENCIES})

set(POSE_SOURCE src/pose_test.cpp src/pose.cpp src/utils.cpp src/display.cpp)
add_executable(pose ${POSE_SOURCE})
target_link_libraries(pose ${DEPENDENCIES})
