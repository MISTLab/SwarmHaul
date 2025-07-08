cmake_minimum_required(VERSION 2.8.12)
project(planning_experiments)
set(CMAKE_CXX_STANDARD 11)
# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Find the GSL package
#find_package(GSL REQUIRED)

# Find the Buzz package
find_package(PkgConfig)
pkg_check_modules(BUZZ REQUIRED buzz)
find_package(Lua 5.3 REQUIRED)
# Find ompl for path planning
#find_package(ompl REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGoS_INCLUDE_DIRS} ${BUZZ_INCLUDE_DIR} ${LUA_INCLUDE_DIR})


# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${BUZZ_LIBRARY_DIRS})

# Compile target
add_library(planning_exp SHARED Planning.h Planning.cpp)

link_directories(${CMAKE_CURRENT_LIST_DIR}/../Hooks_src/build)

find_library(LIB_connecivity NAMES libconnectivity_controller libconnectivity_controller.so)

target_link_libraries(planning_exp
  ${LIB_PROFILER}
  buzz
  ${ARGoS_LIBRARIES}
)
#  ompl)

# Install stuff
install(TARGETS planning_exp DESTINATION ${ARGOS_PREFIX}/lib/argos3)

