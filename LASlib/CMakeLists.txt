project(LASlib)

FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(LASlib STATIC ${SRCS})
target_include_directories(LASlib PUBLIC ${PROJECT_SOURCE_DIR}/include)