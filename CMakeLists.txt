cmake_minimum_required(VERSION 2.8)
project(vslam)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#set(CMAKE_CXX_FLAGS_REALSE "$ENV{CXXFLAGS} -std=c++0x -O3 _Wall")

aux_source_directory(./src DIR_SRCS)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	include
	3rdparty/sophus
	${CERES_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

message(${DIR_SRCS}) 

add_executable(vslam_bin ${DIR_SRCS})
target_link_libraries(vslam_bin ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(vslam ${DIR_SRCS})

