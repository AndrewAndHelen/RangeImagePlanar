cmake_minimum_required(VERSION 3.0.0)
project(RANGE_IMAGE CXX)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
	message(FATAL_ERROR "EIGEN3 not found.")
else()
	message("EIGEN3 library found.${EIGEN3_INCLUDE_DIR}")
endif()
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})

find_package(GDAL REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

file(GLOB HEADS ${PROJECT_SOURCE_DIR}/include/*.h ${PROJECT_SOURCE_DIR}/include/*.hpp)
file(GLOB SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(${PROJECT_NAME} ${SRCS} ${HEADS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include 
${EIGEN_INCLUDE_DIR}
${GDAL_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${CMAKE_SOURCE_DIR}/LASlib/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${GDAL_LIBRARY}
${OpenCV_LIBS}
${PCL_common_LIBRARIES}
LASlib)