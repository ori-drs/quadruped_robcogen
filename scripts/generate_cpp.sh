#!/bin/bash

# Arg 1 is  the name of the robot (e.g., "anymal")
ROBOT_NAME=$1
# Arg 2 is the model of the robot (e.g., "boxy_")
# note the presence of the underscore to concatenate 
# into ${ROBOT_NAME}_${ROBOT_MODEL}description -> anymal_boxy_description
# e.g. if ${ROBOT_NAME} is "vision60" and ${ROBOT_MODEL} is empty,
# ${ROBOT_NAME}_${ROBOT_MODEL}description -> vision60_description
ROBOT_MODEL=$2

# save the folder of the specific robot we generate the code for
ROBOT_DIR=$(rospack find ${ROBOT_NAME}_robcogen)
QUADRUPED_DIR=$(rospack find quadruped_robcogen)
ROBCOGEN_DIR=${QUADRUPED_DIR}/external/robcogen


# remove previously generated header files
rm -f ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/*

# generate the ${ROBOT_NAME} URDF from xacro
echo "Generating \"${ROBOT_NAME}.urdf\" from \"${ROBOT_NAME}.urdf.xacro\" ..."
xacro $(rospack find ${ROBOT_NAME}_${ROBOT_MODEL}description)/urdf/${ROBOT_NAME}.urdf.xacro > ${ROBOT_DIR}/config/${ROBOT_NAME}.urdf

# generate the RobCoGen robot model files from the URDF
echo "Generating \"${ROBOT_NAME}.kindsl\" from \"${ROBOT_NAME}.urdf\" ... "
${QUADRUPED_DIR}/external/urdf2kindsl/urdf2kindsl.py --prune-fixed-joints --lump-inertia -o ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl ${ROBOT_DIR}/config/${ROBOT_NAME}.urdf

# modify the RobCoGen robot model file to set the robot as floating base (otherwise some functions will not be generated)
echo "Setting robot to floating base ..."
sed -i 's/RobotBase base {/RobotBase base floating {/g' ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl

# move to the RobCoGen executable folder
cd ${ROBCOGEN_DIR}

echo "Setting the cpp.cfg path inside core.cfg ..."
grep -qxF 'generator.configfile.cpp = cpp.cfg"' ${ROBCOGEN_DIR}/core.cfg || printf "%s\n" "generator.configfile.cpp = ${ROBCOGEN_DIR}/cpp.cfg" >> ${ROBCOGEN_DIR}/core.cfg

# generate the C++ code inside the /tmp/gen system folder
echo "Generating code from \"${ROBOT_NAME}.kindsl\" ..."

echo "${ROBCOGEN_DIR}/robcogen.sh ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl ${ROBOT_DIR}/config/${ROBOT_NAME}.dtdsl"

${ROBCOGEN_DIR}/robcogen.sh ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl ${ROBOT_DIR}/config/${ROBOT_NAME}.dtdsl ${ROBCOGEN_DIR}/core.cfg

# move back to the specific robot folder
cd ${ROBOT_DIR}

# copy the generated C++ files from /tmp/gen into the include and src folders
echo "Copying C++ files into \"./${ROBOT_NAME}_robcogen\""
cp /tmp/gen/cpp/*.h ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/
cp /tmp/gen/cpp/*.cpp ${ROBOT_DIR}/src/

# modify the jacobians.h header file that otherwise would cause compilation errors
echo "Fixing namespaces in jacobians.h ..."
sed -i '12inamespace internal {' ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h
sed -i '29i} // namespace internal\n' ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h
sed -i 's/Parameters params;/internal::Parameters params;/g' ${ROBOT_DIR}/include/${ROBOT_NAME}_robcogen/jacobians.h

# remove the URDF and RobCoGen robot model files
#echo "Removing files..."
rm -f ${ROBOT_DIR}/config/${ROBOT_NAME}.urdf 
rm -f ${ROBOT_DIR}/config/${ROBOT_NAME}.kindsl

