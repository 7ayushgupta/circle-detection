cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
project(linkwiz)
find_package(Boost REQUIRED)
find_package(PCL 1.2 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file( GLOB LIB_SOURCES lib/*.cpp )
add_library(linkwiz ${LIB_SOURCES})#${PCL_LIBRARIES})

add_executable (convertFile lib/convertFile.cpp)
target_link_libraries (convertFile linkwiz ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (findConcaveHull lib/findConcaveHull.cpp)
target_link_libraries (findConcaveHull linkwiz ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (shiftCentreToCentroid lib/shiftCentreToCentroid.cpp)
target_link_libraries (shiftCentreToCentroid linkwiz ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (findCircles lib/findCircles.cpp)
target_link_libraries (findCircles linkwiz ${PCL_LIBRARIES} ${Boost_LIBRARIES})