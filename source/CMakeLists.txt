## header-start
################################################################################
##
## \file      CMakeList.txt (base folder)
## \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
##
################################################################################
## header-end
##

cmake_minimum_required(VERSION 3.22)

set(PRJNAME project2)
set(PRJVERSION 1.0)

# cmake-lint: disable=C0301


find_package(Git REQUIRED)
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short=12 HEAD OUTPUT_VARIABLE COMMITID OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND "${GIT_EXECUTABLE}" status --short COMMAND fgrep -c " M " OUTPUT_VARIABLE GITCHANGEDFILE OUTPUT_STRIP_TRAILING_WHITESPACE)
set(PRJFULLID "${COMMITID}+${GITCHANGEDFILE}")

project(
    ${PRJNAME}
    VERSION ${PRJVERSION}
    DESCRIPTION "Computing FK and IK for robotic arms"
    LANGUAGES CXX)
    message(STATUS "Project: ${PRJNAME}, Version: ${PRJVERSION}, CommitId: ${PRJFULLID}"
)


set(CMAKE_DEBUG_POSTFIX ".d")
set(CMAKE_SANITIZE_POSTFIX ".s")


#if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
#  message("Sanitize: Setting the debug postfix to .s")
#endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(TARGET1 robot_kinematic)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Werror)
add_compile_options(-DFMT_ENFORCE_COMPILE_STRING)

SET(CMAKE_CXX_FLAGS_SANITIZE
    "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=leak -fno-omit-frame-pointer"
    CACHE STRING "Flags used by the C++ compiler for leak sanitized build type or configuration."
    FORCE)

SET(CMAKE_EXE_LINKER_FLAGS_SANITIZE
    "-fsanitize=leak"
    CACHE STRING "Flags used for linking binaries during sanitize builds."
    FORCE)

MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_SANITIZE
    CMAKE_EXE_LINKER_FLAGS_SANITIZE )



if (WIN32)
  message("VS-Code using explicit folders for fmt, toml11 and glm")
else()
  message("Linux  using implicit packages folders for fmt, toml11 and glm")
  find_package(fmt CONFIG REQUIRED)
  find_package(toml11 CONFIG REQUIRED)
  find_package(glm CONFIG REQUIRED)
endif()


add_subdirectory(src)

message(STATUS "Current build type is : ${CMAKE_BUILD_TYPE}")


if(ENABLE_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

#include(src)

#add_executable(${PRJNAME} ${PROJECT_SOURCES})
#set_target_properties(${PRJNAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#target_link_libraries(${PRJNAME} glm::glm)

#target_compile_definitions(${PRJNAME} PUBLIC VERSION=${PRJVERSION})
#set_target_properties(${PRJNAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#target_link_libraries(${PRJNAME} glm::glm)

#### additional help on CMakeLists.txt
#
# refer to https://gitlab.com/CLIUtils/modern-cmake/-/tree/master/examples/extended-project 
# for more details on this CMAKE
#

