cmake_minimum_required(VERSION 3.0)

project(streetsmarts)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")


find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(GLUT)
find_package(PCL)
find_package (Eigen3)
find_package(Open3D)
find_package(realsense2)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-unused-result)

add_definitions(-DUNIX)
add_definitions(-O3)
add_definitions(${PCL_DEFINITIONS})

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
  ${PCL_INCLUDE_DIRS}
) 

link_directories(${Open3D_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBS})

set(DEPENDENCIES 
	${GLUT_LIBRARY}
	${realsense2_LIBRARY} 
	${OPENGL_LIBRARIES} 
	${OpenCV_LIBS} 
	glfw 
	${Open3D_LIBRARIES} 
	Eigen3::Eigen 
	/home/ben/Open3D/build/lib/libCuda.a
        ${PCL_LIBRARIES} 
)


set(CLUSTER_SOURCE src/analyze/cluster.cpp src/config.cpp)
add_executable(cluster ${CLUSTER_SOURCE})
target_link_libraries(cluster ${DEPENDENCIES})

set(ANALYZE_SOURCE src/analyze/don.cpp src/utils.cpp src/config.cpp)
add_executable(analyze ${ANALYZE_SOURCE})
target_link_libraries(analyze ${DEPENDENCIES})

#set(KITTI_SOURCE src/kitti.cpp src/utils.cpp src/config.cpp)
#add_executable(kitti ${KITTI_SOURCE})
#target_link_libraries(kitti ${DEPENDENCIES})

set(CAPTURE_SOURCE src/capture.cpp src/utils.cpp src/config.cpp)
add_executable(capture ${CAPTURE_SOURCE})
target_link_libraries(capture ${DEPENDENCIES})
