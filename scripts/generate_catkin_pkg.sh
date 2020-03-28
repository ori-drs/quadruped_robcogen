#!/bin/bash

if [ $# -lt  1 ]
  then
    echo "ERROR: missing required argument ROBOT_NAME. Exiting ..."
    echo "USAGE: generate_quadruped_catkin_pkg.sh ROBOT_NAME [ ROBOT_DESCRIPTION_PKG_NAME ] [ ROBOT_XACRO_NAME ] [ LICENSE ]"
    exit 1
fi

# Arg 1 is  the name of the robot (e.g., "anymal")
ROBOT_NAME=$1

if [ $# -gt 1 ]
  then
    # Arg 2 is the package name of the robot description (e.g., "anymal_description")
    ROBOT_DESCRIPTION_PKG_NAME=$2
  else
    echo "WARNING: missing optional argument ROBOT_DESCRIPTION_PKG_NAME."
    echo "Using default: ${ROBOT_NAME}_description"
    ROBOT_DESCRIPTION_PKG_NAME=${ROBOT_NAME}_description
fi

if [ $# -gt 2 ]
  then
    # Arg 3 is the name of the root xacro file of the description, without extension (e.g., anymal)
    ROBOT_XACRO_NAME=$3
  else
    echo "WARNING: missing optional argument ROBOT_XACRO_NAME."
    echo "Using default: ${ROBOT_NAME}"
    ROBOT_XACRO_NAME=${ROBOT_NAME}
fi

if [ $# -gt 3 ]
  then
    # Arg 4 is the package license
    LICENSE=$4
  else
    echo "WARNING: missing optional argument LICENSE."
    echo "Using default: Proprietary"
    LICENSE=Proprietary
fi

# Create the package
catkin create pkg --catkin-deps quadruped_robcogen ${ROBOT_DESCRIPTION_PKG_NAME} pronto_quadruped_commons --description "Kinematics and Dynamics libraries for the ${ROBOT_NAME} robot, automatically generated through RobCoGen" --license ${LICENSE} -- ${ROBOT_NAME}_robcogen

# Create subdirectories
mkdir ${ROBOT_NAME}_robcogen/src
mkdir ${ROBOT_NAME}_robcogen/config
mkdir -p ${ROBOT_NAME}_robcogen/include/${ROBOT_NAME}_robcogen

# Copy template for CMakeLists.txt
cp $(rospack find quadruped_robcogen)/config/CMakeLists.txt.in ${ROBOT_NAME}_robcogen/CMakeLists.txt

# Replace variable names in the CMakeLists template we just copied
sed -i "s/@ROBOT_NAME@/${ROBOT_NAME}/g" ${ROBOT_NAME}_robcogen/CMakeLists.txt
sed -i "s/@ROBOT_DESCRIPTION_PKG_NAME@/${ROBOT_DESCRIPTION_PKG_NAME}/g" ${ROBOT_NAME}_robcogen/CMakeLists.txt
sed -i "s/@ROBOT_XACRO_NAME@/${ROBOT_XACRO_NAME}/g" ${ROBOT_NAME}_robcogen/CMakeLists.txt

