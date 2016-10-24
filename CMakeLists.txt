cmake_minimum_required(VERSION 2.8)
project(vslam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall -pthread")

aux_source_directory(./src DIR_SRCS)
message(${DIR_SRCS})
message(${PROJECT_SOURCE_DIR})
#set(LIBRARY_OUTPUT_PATH )

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	include
	3rdparty/sophus
	${CERES_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)



add_executable(vslam_bin ${DIR_SRCS})
target_link_libraries(vslam_bin ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

#add_library(vslam SHARED ${DIR_SRCS})

