#!/bin/sh

ROBCOGEN_DIR=$(rospack find quadruped_robcogen)/external/robcogen
CLASSPATH=`find ${ROBCOGEN_DIR}/lib/ -name '*.jar' | tr '\n' ':'`

echo "java -Dlog4j.configuration=file:log4j.cfg -classpath ${CLASSPATH}:${ROBCOGEN_DIR}/bin/ iit.robcogen.Generator $1 $2 $3"
java -Dlog4j.configuration=file:${ROBCOGEN_DIR}/log4j.cfg -classpath ${CLASSPATH}:${ROBCOGEN_DIR}/bin/ iit/robcogen/Generator $1 $2 $3

