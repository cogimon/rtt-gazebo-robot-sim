cmake_minimum_required(VERSION 2.8.3)
project(rtt-gazebo-robot-sim)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(gazebo 6.5 REQUIRED)
message(STATUS "Found GAZEBO_VERSION: ${GAZEBO_VERSION}")
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()

find_package(SDFormat REQUIRED)

#message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
#message(STATUS "GAZEBO_LIBRARY_DIR: ${GAZEBO_LIBRARY_DIR}")
#message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")


find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(srdfdom_advr REQUIRED)
message(STATUS "srdfdom_advr_INCLUDE_DIRS: ${srdfdom_advr_INCLUDE_DIRS}")
message(STATUS "srdfdom_advr_LIBRARY_DIRS: ${srdfdom_advr_LIBRARY_DIRS}")

find_package(urdf REQUIRED)
message(STATUS "urdf_INCLUDE_DIRS: ${urdf_INCLUDE_DIRS}")
message(STATUS "urdf_LIBRARY_DIRS: ${urdf_LIBRARY_DIRS}")

find_package(XBotCoreModel REQUIRED)
message(STATUS "XBotCoreModel_INCLUDE_DIRS: ${XBotCoreModel_INCLUDE_DIRS}")
message(STATUS "XBotCoreModel_LIBRARY_DIR: ${XBotCoreModel_LIBRARY_DIR}")

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${srdfdom_advr_LIBRARY_DIRS}
  ${urdf_LIBRARY_DIRS}
  ${XBotCoreModel_LIBRARY_DIR}
  ${RST-RT_LIBRARY_DIR}
  ${RST-RT_LIBRARY_DIRS}
  ${TINYXML_LIBRARY_DIR}
  ${}
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${RST-RT_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
    ${srdfdom_advr_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS}
    ${XBotCoreModel_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/rtt-gazebo-robot-sim.cpp src/rtt-gazebo-robot-sim_hooks.cpp src/kinematic_chain.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${SDFormat_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${urdf_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
)

orocos_generate_package(INCLUDE_DIRS include)
