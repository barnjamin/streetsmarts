cmake_minimum_required(VERSION 3.0)

project(streetsmarts)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib") 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")


find_package(OpenGL)
find_package(PCL)
set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} glfw)

message(STATUS "${PCL_LIBRARIES}")
		
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${PCL_INCLUDE_DIRS}
) 

set(APP_SOURCES src/pcl.cpp)

set(LIB_SOURCES src/street.cpp)

# creates ./build/bin/runner
add_executable(${PROJECT_NAME} ${APP_SOURCES})

target_link_libraries(streetsmarts ${DEPENDENCIES})

# shared library
set(LIBRARY_NAME World)
add_library(${LIBRARY_NAME} SHARED ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBRARY_NAME})

