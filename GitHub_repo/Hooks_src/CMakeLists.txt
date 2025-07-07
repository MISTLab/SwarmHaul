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
# link_directories(${ARGoS_LIBRARY_DIRS} ${BUZZ_LIBRARY_DIRS})
# include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${BUZZ_INCLUDE_DIR})

#include_directories(${OMPL_INCLUDE_DIRS})
#link_directories(${OMPL_LIBRARY_DIR})

# Find Lua


# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${BUZZ_LIBRARY_DIRS})

# Compile target
add_library(connectivity_controller SHARED virtual_stigmergy2.c virtual_stigmergy2.h controller.h controller.cpp svg_image.cpp svg_image.h)

# path_existance_checking.cpp path_existance_checking.h 
#            )

target_link_libraries(connectivity_controller
  buzz
  ${ARGoS_LIBRARIES}
  )
 # ompl)

# Install stuff
install(TARGETS connectivity_controller DESTINATION ${ARGOS_PREFIX}/lib/argos3)

