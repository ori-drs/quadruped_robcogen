set(CURRENT_MACRO_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

macro(robcogen ROBOT_NAME)

  #store the location of the robcogen executable
  set(ROBCOGEN_DIR ${CURRENT_MACRO_DIR}/../external/robcogen)

  #store the required argument as variable for later use
  set(ROBOT_NAME ${ROBOT_NAME})
  message(STATUS "ROBOT_NAME = ${ROBOT_NAME}")

  # capture the optional arguments ROBOT_DESCRIPTION_PKG_NAME and ROBOT_XACRO_NAME
  set (EXTRA_MACRO_ARGS ${ARGN})
  list(LENGTH EXTRA_MACRO_ARGS NUM_EXTRA_ARGS)
  if (${NUM_EXTRA_ARGS} GREATER 0)
    list(GET EXTRA_MACRO_ARGS 0 ROBOT_DESCRIPTION_PKG_NAME)
    message(STATUS "ROBOT_DESCRIPTION_PKG_NAME = ${ROBOT_DESCRIPTION_PKG_NAME}")
  endif ()

  if (${NUM_EXTRA_ARGS} GREATER 1)
    list(GET EXTRA_MACRO_ARGS 1 ROBOT_XACRO_NAME)
    message(STATUS "ROBOT_XACRO_NAME = ${ROBOT_XACRO_NAME}")
  endif ()

  #set the name of the library that is going to be created
  set(LIB_NAME ${ROBOT_NAME}_robcogen)
  
  #define the list of headers and sources to be generated in the target package
  set(HEADERS ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/declarations.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/forward_dynamics.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/inverse_dynamics.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/joint_data_map.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/kinematics_parameters.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/miscellaneous.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/rbd_types.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/transforms.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/dynamics_parameters.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/inertia_properties.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/jsim.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/link_data_map.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/model_constants.h
              ${CMAKE_SOURCE_DIR}/include/${ROBOT_NAME}_robcogen/traits.h)

  set(SOURCES ${CMAKE_SOURCE_DIR}/src/transforms.cpp
              ${CMAKE_SOURCE_DIR}/src/jacobians.cpp
              ${CMAKE_SOURCE_DIR}/src/jsim.cpp
              ${CMAKE_SOURCE_DIR}/src/inverse_dynamics.cpp
              ${CMAKE_SOURCE_DIR}/src/forward_dynamics.cpp
              ${CMAKE_SOURCE_DIR}/src/inertia_properties.cpp
              ${CMAKE_SOURCE_DIR}/src/miscellaneous.cpp)



  # Add library
  add_library(${LIB_NAME} SHARED ${SOURCES})

  # copy the transforms file into the target package with the name of the robot in it
  # NOTE: this file assumes there are four feet named LF_FOOT RF_FOOT LH_FOOT and RH_FOOT 
  configure_file(${CURRENT_MACRO_DIR}/../config/robot.dtdsl ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.dtdsl)
  configure_file(${CURRENT_MACRO_DIR}/../external/robcogen/core.cfg.in ${CURRENT_MACRO_DIR}/../external/robcogen/core.cfg)

  add_custom_command(OUTPUT ${HEADER} ${SOURCES} ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.kindsl 
                     COMMAND ${CURRENT_MACRO_DIR}/../scripts/generate_cpp.sh
                     ARGS ${ROBOT_NAME} ${CMAKE_SOURCE_DIR} ${ROBOT_DESCRIPTION_PKG_NAME} ${ROBOT_XACRO_NAME}
                     DEPENDS ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.dtdsl
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  # remove the sources so robcogen generates the code always
  add_custom_command(TARGET ${LIB_NAME} POST_BUILD
                     COMMAND rm ${SOURCES} ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.dtdsl ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.kindsl
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})


  add_custom_target(${ROBOT_NAME}_robcogen_cpp 
                    DEPEND ${HEADERS} ${SOURCES} ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.kindsl)

  catkin_package(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
                              ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
                 LIBRARIES ${LIB_NAME}
                 DEPENDS EIGEN3
                 EXPORTED_TARGETS ${ROBOT_NAME}_robcogen_cpp)
endmacro()

