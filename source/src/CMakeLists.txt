## header-start
################################################################################
##
## \file      CMakeList.txt (src folder)
## \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
##
################################################################################
## header-end
##

set(PROJECT_SOURCES
  logger.h
  context.h
  util.h
  bbox3D.h
  ikparams.h
  glm_util.h  glm_util.cpp
  facet.h  facet.cpp
  stlmesh.h  stlmesh.cpp
  joint.h  joint.cpp
  link.h link.cpp
  poses.h poses.cpp
  robot.h  robot.cpp
  main.cpp
)

#if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
#  add_compile_options(-DFMT_ENFORCE_COMPILE_STRING -g -fsanitize=leak)
#  add_link_options(-fsanitize=leak)
#endif()

add_executable(${TARGET1} ${PROJECT_SOURCES})
target_compile_definitions(${TARGET1} PUBLIC PROG_VERSION="${PRJVERSION}")
target_compile_definitions(${TARGET1} PUBLIC PROG_NAME="${TARGET1}")
target_compile_definitions(${TARGET1} PUBLIC PROG_GITID="${PRJFULLID}")

if (WIN32)
  target_include_directories(${TARGET1} PRIVATE "$ENV{APPDATA}/fmt/include")
  target_include_directories(${TARGET1} PRIVATE "$ENV{APPDATA}/tomlplusplus/include")
  target_include_directories(${TARGET1} PRIVATE "$ENV{APPDATA}/glm/include")

  target_link_directories(${TARGET1} PRIVATE "$ENV{APPDATA}/fmt/lib")
  target_link_libraries(${TARGET1} fmt )

else()
  target_link_libraries(${TARGET1} glm::glm)
  target_link_libraries(${TARGET1} toml11::toml11)
  target_link_libraries(${TARGET1} fmt)
endif()
target_link_libraries(${TARGET1} m)


#target_include_directories(${TARGET1 PRIVATE /absolute/path

#if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
#  set_target_properties(${TARGET1} PROPERTIES CMAKE_SANITIZE_POSTFIX ${CMAKE_SANITIZE_POSTFIX})
#endif()

#if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#  set_target_properties(${TARGET1} PROPERTIES CMAKE_DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#endif()

#set_target_properties(${TARGET1} PROPERTIES CMAKE_DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
#set_target_properties(${TARGET1} PROPERTIES CMAKE_SANITIZE_POSTFIX ${CMAKE_SANITIZE_POSTFIX})

add_custom_command(TARGET ${TARGET1} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          $<TARGET_FILE:${TARGET1}> ${PROJECT_SOURCE_DIR}/${TARGET1}.exe
  COMMENT "Created ${PROJECT_SOURCE_DIR}/${TARGET1}.exe"
)
