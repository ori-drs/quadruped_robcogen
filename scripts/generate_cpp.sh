#!/bin/bash

if [ $# -lt  2 ]
  then
    echo "ERROR: missing required arguments ROBOT_NAME and ROBOT_DIR. Exiting ..."
    exit 1
fi

# Arg 1 is  the name of the robot (e.g., "anymal")
ROBOT_NAME=$1

# Arg 2 is the path of the specific robot package invoking this script (e.g., anymal_robcogen)
ROBOT_DIR=$2

if [ $# -gt 2 ]
  then
    # Arg 3 is the package name of the robot description (e.g., "anymal_description")
    ROBOT_DESCRIPTION_PKG_NAME=$3
  else
    echo "WARNING: missing optional argument ROBOT_DESCRIPTION_PKG_NAME."
    echo "Using default: ${ROBOT_NAME}_description"
    ROBOT_DESCRIPTION_PKG_NAME=${ROBOT_NAME}_description
fi

if [ $# -gt 3 ]
  then
    # Arg 4 is the name of the root xacro file of the description, without extension (e.g., anymal)
    ROBOT_XACRO_NAME=$4
  else
    echo "WARNING: missing optional argument ROBOT_XACRO_NAME."
    echo "Using default: ${ROBOT_NAME}"
    ROBOT_XACRO_NAME=${ROBOT_NAME}
fi

if [ $# -gt 4 ]
  then
    # Arg 5 is a string of xacro command line arguments concatenated with the '@@@' pattern
    # so that all the xacro arguments appear as one string
    XACRO_ARGS=$5
    # replace the "@@@" pattern with a space, so we can pass the arguments to xacro
    XACRO_ARGS=${XACRO_ARGS//@@@/" "}
fi
# save the folder of the specific robot we generate the code for
QUADRUPED_DIR=$(rospack find quadruped_robcogen)
ROBCOGEN_DIR=${QUADRUPED_DIR}/external/robcogen


# remove previously generated header files
rm -f ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/*

# generate the ${ROBOT_NAME} URDF from xacro
echo "Generating \"${ROBOT_NAME}.urdf\" from \"${ROBOT_XACRO_NAME}.urdf.xacro\" ..."
xacro $(rospack find ${ROBOT_DESCRIPTION_PKG_NAME})/urdf/${ROBOT_XACRO_NAME}.urdf.xacro ${XACRO_ARGS} > ${ROBOT_DIR}/config/${ROBOT_NAME}.urdf

# generate the RobCoGen robot model files from the URDF
echo "Generating \"${ROBOT_NAME}.kindsl\" from \"${ROBOT_NAME}.urdf\" ... "
${QUADRUPED_DIR}/external/urdf2kindsl/urdf2kindsl.py --prune-fixed-joints --lump-inertia -o ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl ${ROBOT_DIR}/config/${ROBOT_NAME}.urdf

# modify the RobCoGen robot model file to set the robot as floating base (otherwise some functions will not be generated)
echo "Setting robot to floating base ..."
sed -i 's/RobotBase base {/RobotBase base floating {/g' ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl

# generate the C++ code inside the /tmp/gen system folder
echo "Generating code from \"${ROBOT_NAME}.kindsl\" ..."
${ROBCOGEN_DIR}/robcogen.sh ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl ${ROBOT_DIR}/config/${ROBOT_NAME}.dtdsl ${ROBCOGEN_DIR}/core.cfg

# copy the generated C++ files from /tmp/gen into the include and src folders
echo "Copying C++ files into \"./${ROBOT_NAME}_robcogen\""
mkdir -p ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/
mkdir -p ${ROBOT_DIR}/src/

cp /tmp/gen/cpp/*.h ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/
cp /tmp/gen/cpp/*.cpp ${ROBOT_DIR}/src/

# modify the jacobians.h header file that otherwise would cause compilation errors
echo "Fixing namespaces in jacobians.h ..."
sed -i '12inamespace internal {' ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h
sed -i '29i} // namespace internal\n' ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h
sed -i 's/Parameters params;/internal::Parameters params;/g' ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h

# remove the URDF and RobCoGen robot model files
echo "Removing files..."
rm -f ${ROBOT_DIR}/config/${ROBOT_NAME}.urdf 
#rm -f ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl
rm -f ${ROBOT_DIR}/maxima.log
rm -f ${ROBOT_DIR}/robcogen.log


