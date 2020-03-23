set(CURRENT_MACRO_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

macro(robcogen ROBOT_NAME ROBOT_MODEL)
  set(LIB_NAME ${ROBOT_NAME}_robcogen)
  
  #store the argument as variable for the configuration below
  set(ROBOT_NAME ${ROBOT_NAME})

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

  add_custom_command(OUTPUT ${HEADERS} ${SOURCES}
                     COMMAND ${CURRENT_MACRO_DIR}/../scripts/generate_cpp.sh
                     ARGS ${ROBOT_NAME} ${ROBOT_MODEL}
                     DEPENDS ${CMAKE_SOURCE_DIR}/config/${ROBOT_NAME}.dtdsl
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  # remove the sources so robcogen generates the code always
  add_custom_command(TARGET ${LIB_NAME} POST_BUILD
                     COMMAND rm ${SOURCES}
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})


  add_custom_target(${ROBOT_NAME}_robcogen_cpp 
                    DEPEND ${HEADERS} ${SOURCES})

  catkin_package(INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
                              ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
                 LIBRARIES ${LIB_NAME}
                 DEPENDS EIGEN3
                 EXPORTED_TARGETS ${ROBOT_NAME}_robcogen_cpp)
endmacro()

