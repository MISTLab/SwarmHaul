cmake_minimum_required(VERSION 2.8.12)
project(planning_experiments)
set(CMAKE_CXX_STANDARD 11)
# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Find the GSL package
#find_package(GSL REQUIRED)

# Find the Buzz package
find_package(PkgConfig)
pkg_check_modules(BUZZ REQUIRED buzz)

# Find ompl for path planning
find_package(ompl REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${BUZZ_INCLUDE_DIR})

include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIR})
# Find Lua
find_package(Lua)
if(Lua_FOUND)
  include_directories(${LUA_INCLUDE_DIR})
endif(Lua_FOUND)

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${BUZZ_LIBRARY_DIRS})

# Compile target
add_library(planning_exp SHARED Planning.h Planning.cpp)

link_directories(/home/vivek/Vivek/Projects/Connectivity/GitHub_repo/Hooks_src/build)

find_library(LIB_connecivity NAMES libconnectivity_controller libconnectivity_controller.so)

target_link_libraries(planning_exp
  ${LIB_PROFILER}
  buzz
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_dynamics3d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_buzz
  ompl)

# Install stuff
install(TARGETS planning_exp DESTINATION ${ARGOS_PREFIX}/lib/argos3)

