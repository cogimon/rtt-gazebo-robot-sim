cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME tests)
project(${PROJECTNAME})

find_package(GTest)
if (NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} )
enable_testing()

add_custom_command( TARGET gain_parser POST_BUILD
               COMMAND ${CMAKE_CTEST_COMMAND}
               MAIN_DEPENDENCY gain_parser
               DEPENDS
               test-parser
               )

SET(TestLibs gain_parser ${GTEST_BOTH_LIBRARIES} -lpthread
   ${TINYXML_LIBRARIES}
   )

ADD_EXECUTABLE(test_parser     test_parser.cpp)
TARGET_LINK_LIBRARIES(test_parser ${TestLibs})
add_dependencies(test_parser GTest-ext gain_parser )

add_test(NAME rtt_gazebo_robot_test_parser COMMAND test_parser)

