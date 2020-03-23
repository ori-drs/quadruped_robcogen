#!/bin/sh
echo "++++++++++++++++++++++++++++++++" `pwd`

ROBCOGEN_DIR=$(rospack find quadruped_robcogen)/external/robcogen
CLASSPATH=`find ${ROBCOGEN_DIR}/lib/ -name '*.jar' | tr '\n' ':'`

echo "------------------------- ${CP}"
echo "java -Dlog4j.configuration=file:log4j.cfg -classpath ${CLASSPATH}:${ROBCOGEN_DIR}/bin/ iit.robcogen.Generator $1 $2"
java -Dlog4j.configuration=file:${ROBCOGEN_DIR}/log4j.cfg -classpath ${CLASSPATH}:${ROBCOGEN_DIR}/bin/ iit/robcogen/Generator $1 $2

rm ${ROBCOGEN_DIR}/maxima.log ${ROBCOGEN_DIR}/robcogen.log
