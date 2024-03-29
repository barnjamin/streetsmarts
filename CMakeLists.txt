cmake_minimum_required(VERSION 3.0)

project(streetsmarts)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
message("mod path: ${CMAKE_MODULE_PATH}")


set(bitness 64)

find_package(CUDA 10.0 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OpenGL)
find_package(GLUT)
find_package(PCL)
find_package(Eigen3)
find_package(Open3D)
find_package(realsense2)
find_package(GeographicLib REQUIRED)
find_package(LibOSRM REQUIRED)
find_package(CURL)
find_package(Boost)

set (extra_boost_components)
find_package(Boost REQUIRED
    COMPONENTS program_options system ${extra_boost_components})

find_package(Threads REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-unused-result)

add_definitions(-DUNIX)
add_definitions(-O3)
add_definitions(${PCL_DEFINITIONS})
add_definitions (${GeographicLib_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})

set(IMGUI /home/ben/librealsense/third-party/imgui)

aux_source_directory(${IMGUI} IMGUI_SOURCES)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${OpenCV_INCLUDE_DIRS}
  ${IMGUI}
  ${relasense_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
  /usr/include/
  /usr/local/include/Open3D
  ${OPENGL_INCLUDE_DIRS}  
  ${GLUT_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  /home/ben/ublox/include/
  /home/ben/serial/include/
  /home/ben/streetsmarts/src/
  /home/ben/streetsmarts/src/gps
  ${CUDA_INCLUDE_DIRS}
  ${IMGUI}
  ${LibOSRM_INCLUDE_DIRS}
  ${CURL_INDLUE_DIRS}
  /home/ben/cc.ublox.commsdsl/build/output/include
  /home/ben/cc.ublox.commsdsl/build/comms_champion/build/install/include/
) 

link_directories(${Open3D_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBS})
link_directories(${IMGUI})
link_directories(${LibOSRM_LIBRARY_DIRS})


set(DEPENDENCIES 
	${GLUT_LIBRARY}
	${realsense2_LIBRARY} 
	${OPENGL_LIBRARIES} 
	${OpenCV_LIBS} 
	glfw 
	${Open3D_LIBRARIES} 
	Eigen3::Eigen 
    /home/ben/Open3D/build/lib/libOpen3DCuda.a
    ${PCL_LIBRARIES} 
    ${GeographicLib_LIBRARIES}
    ${CUDA_LIBRARIES}
    /home/ben/ublox/lib/libublox.a
    /home/ben/serial/lib/libserial.a
    /usr/lib/x86_64-linux-gnu/libcurand.so
    ${LibOSRM_LIBRARIES}
    ${CURL_LIBRARIES}
    ${LibOSRM_DEPENDENT_LIBRARIES}
)



set(CLIENT_SOURCE 
    src/client.cpp 
    src/utils.cpp 
    src/config.cpp 
    src/handlers.hpp 
    src/pose/pose.cpp 
    src/fragments.hpp 
    src/gps.cpp
    )
add_executable(client ${CLIENT_SOURCE})
target_link_libraries(client ${DEPENDENCIES})

set(FRAGMENT_SOURCE src/make_fragments.cpp src/utils.cpp src/config.cpp src/fragments.hpp)
add_executable(make_fragments ${FRAGMENT_SOURCE})
target_link_libraries(make_fragments ${DEPENDENCIES})

set(REFINE_SOURCE src/refine.cpp src/utils.cpp src/config.cpp)
add_executable(refine ${REFINE_SOURCE})
target_link_libraries(refine ${DEPENDENCIES})

set(INTEGRATE_SOURCE src/integration.cpp src/utils.cpp src/config.cpp src/integrate.hpp)
add_executable(integrate ${INTEGRATE_SOURCE})
target_link_libraries(integrate ${DEPENDENCIES})

set(ANALYZE_SOURCE src/analyze/don.cpp src/utils.cpp src/config.cpp)
add_executable(analyze ${ANALYZE_SOURCE})
target_link_libraries(analyze ${DEPENDENCIES})

set(CLUSTER_SOURCE src/analyze/cluster.cpp src/config.cpp src/utils.cpp)
add_executable(cluster ${CLUSTER_SOURCE})
target_link_libraries(cluster ${DEPENDENCIES})

#set(SELECT_SOURCE src/analyze/select.cpp)
#add_executable(select ${SELECT_SOURCE})
#target_link_libraries(select ${DEPENDENCIES})

#set(KITTI_SOURCE src/kitti.cpp src/utils.cpp src/config.cpp)
#add_executable(kitti ${KITTI_SOURCE})
#target_link_libraries(kitti ${DEPENDENCIES})

#set(SUPERPIX_SOURCE src/superpix.cpp)
#add_executable(superpix ${SUPERPIX_SOURCE})
#target_link_libraries(superpix ${DEPENDENCIES})

#set(SEGMENT_SOURCE src/experiments/segment.cpp src/config.cpp src/utils.cpp)
#add_executable(segment ${SEGMENT_SOURCE})
#target_link_libraries(segment ${DEPENDENCIES})

#set(CAPTURE_SOURCE src/capture_frames.cpp src/utils.cpp src/config.cpp)
#add_executable(capture_frames ${CAPTURE_SOURCE})
#target_link_libraries(capture_frames ${DEPENDENCIES})

#set(NORMAL_SOURCE src/experiments/normals.cpp src/utils.cpp src/config.cpp)
#add_executable(normals ${NORMAL_SOURCE})
#target_link_libraries(normals ${DEPENDENCIES})

#set(T265_SOURCE src/t265.cpp src/utils.cpp)
#add_executable(t265 ${T265_SOURCE})
#target_link_libraries(t265 ${DEPENDENCIES})

#set(GUI_SOURCE src/gui/main.cpp ${IMGUI_SOURCES})
#add_executable(gui ${GUI_SOURCE})
#include_directories(gui ${IMGUI})
#target_link_libraries(gui ${DEPENDENCIES})

set(ROAD_SOURCE src/experiments/make_road.cpp src/utils.cpp src/config.cpp src/fragments.hpp)
add_executable(make_road ${ROAD_SOURCE})
target_link_libraries(make_road ${DEPENDENCIES})

#set(ROUTE_SOURCE src/route.cpp src/utils.cpp src/config.cpp)
#add_executable(route ${ROUTE_SOURCE})
#target_link_libraries(route ${DEPENDENCIES})

#set(MESH_SOURCE src/experiments/mesh.cpp src/utils.cpp src/config.cpp)
#add_executable(mesh ${MESH_SOURCE})
#target_link_libraries(mesh ${DEPENDENCIES})
#
#set(VPG_SOURCE src/experiments/view_pg.cpp src/utils.cpp src/config.cpp)
#add_executable(vpg ${VPG_SOURCE})
#target_link_libraries(vpg ${DEPENDENCIES})
#
#set(VRGBD_SOURCE src/experiments/view_rgbd.cpp src/utils.cpp src/config.cpp)
#add_executable(vrgbd ${VRGBD_SOURCE})
#target_link_libraries(vrgbd ${DEPENDENCIES})
#
#set(DRGBD_SOURCE src/experiments/RGBDOdometryCuda.cpp src/utils.cpp src/config.cpp)
#add_executable(drgbd ${DRGBD_SOURCE})
#target_link_libraries(drgbd ${DEPENDENCIES})

set(GPS_SOURCE src/gps_test.cpp src/utils.cpp src/config.cpp src/gps.cpp)
add_executable(gps ${GPS_SOURCE})
target_link_libraries(gps ${DEPENDENCIES})

