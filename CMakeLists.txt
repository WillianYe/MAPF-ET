cmake_minimum_required (VERSION 3.7.1)

project(cbs_et)

set(CMAKE_EXPORT_COMPILE_COMMANDS True)
SET(CMAKE_BUILD_TYPE "RELEASE")
# SET(CMAKE_BUILD_TYPE "DEBUG")

# IF(NOT CMAKE_BUILD_TYPE)
#     SET(CMAKE_BUILD_TYPE "RELEASE")
# ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories("CBS_ET/inc" "path_coordination/inc")
file(GLOB CBSET_SOURCES "CBS_ET/src/*.cpp")
file(GLOB COORDINATION_SOURCES "path_coordination/src/*.cpp")

add_executable(cbs_et ${CBSET_SOURCES} ${COORDINATION_SOURCES} main.cpp)

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(cbs_et ${Boost_LIBRARIES})
