cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(viewer)

include_directories(../)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

pkg_check_modules(CHOREONOID REQUIRED choreonoid-body-plugin)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

set(srcs ../WalkPlanner.cpp ../RobotCnoid.cpp ../StepSequence.cpp ../creekGeometry.cpp)

add_executable (main main.cpp ${srcs})
target_link_libraries (main ${PCL_LIBRARIES} ${CHOREONOID_LIBRARIES})
