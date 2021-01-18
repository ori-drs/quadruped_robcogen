# quadruped_robcogen

This package allows to generate the C++ [RobCoGen](https://robcogenteam.bitbucket.io/) code for any quadruped robot
with 12 active DoF. The code includes forward/inverse kinematics and dynamics.

Under the hood the code is generated in three steps:
1. the xacro file is converted into URDF via the standard [`xacro`](http://wiki.ros.org/xacro) command
2. the URDF is converted into a KINDSL file (the RobCoGen robot modelformat) with the [`urdf2kindsl`](https://bitbucket.org/robcogenteam/urdf2kindsl/src/master/) command
3. the `${ROBOT_NAME}.kindsl` model and the transforms file `${ROBOT_NAME}.dtdsl` (where `${ROBOT_NAME}` is the name of your robot) are passed to the RobCoGen executable to make the CPPs

All the above programs are either copied locally into this repo or available from ROS.

## Dependencies
This repository contains a slightly modified copy (to bypass the interactive menu) of the RobCoGen executable (version 5.0.1), which is written in Java.
The following dependencies are therefore required to run the program correctly:
- Java JRE
- ANT
- Ivy
- Maxima
- gcc

On Ubuntu, these can be installed as follows:
```
sudo apt install openjdk-8-jre  ant ivy maxima gcc
```
**NOTE**: Ivy has a faulty installation. 
If you experience problems, you can fix with a symlink:
```
sudo ln -s /usr/share/java/ivy.jar /usr/share/ant/lib/ivy.jar 
```

The generated code also depends on the Rigid Body Dynamics (RBD) interfaces available [as part of the pronto project](https://github.com/ori-drs/pronto/tree/master/pronto_quadruped_commons).  This is a catkinized copy of the original code [here](https://bitbucket.org/robcogenteam/cpp-iitrbd/src/master/).

## The `robcogen` CMake macro
The [robcogen.cmake](cmake/robcogen.cmake) file defines the macro to be used within your custom `CMakeLists.txt` to generate the code for your robot. It takes two arguments:
- `ROBOT_NAME` is the name of your robot, e.g., `fido` 
- `ROBOT_DESCRIPTION_PKG_NAME` (optional) is the name of the package where to fetch the xacro description of the robot. *Default:*  `${ROBOT_NAME}_description`
- `ROBOT_XACRO_NAME` (optional) name of the xacro file to be parsed, without extension. *Default:* `${ROBOT_NAME}` 
- `XACRO_ARGS_FILE` (optional) absolute path of a file containing the arguments to be passed to the `xacro` command, one per line (see below)
## Build the Code for Your Robot
To generate the code for your robot, create a catkin package that depends on 
`quadruped_robcogen` and use the provided the `robcogen` cmake macro to build your code.

You can easily do this by running the provided script `generate_catkin_pkg.sh`, which takes the same arguments as the cmake macro described above.
For example, the following command
```
rosrun quadruped_robcogen generate_catkin_pkg.sh fido
``` 

Would produce the  a `CMakeLists.txt` for a robot called `fido`:

```
cmake_minimum_required(VERSION 2.8.3)
project(fido_robcogen)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS pronto_quadruped_commons
                                        quadruped_robcogen
                                        fido_description)

set(LIB_NAME ${PROJECT_NAME})

find_package(Eigen3)

catkin_package(INCLUDE_DIRS include
               LIBRARIES fido_robcogen
               CATKIN_DEPENDS pronto_quadruped_commons
                              quadruped_robcogen
                              fido_description)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

include_directories(./include)
include_directories(./include/${PROJECT_NAME})
include(${quadruped_robcogen_SOURCE_PREFIX}/cmake/robcogen.cmake)

# call the macro to generate the C++ code
robcogen(fido fido_description fido)

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
```
## Xacro arguments
If your xacro file has arguments, you can pass them to the `robcogen` cmake macro as a file located in the `config` folder of your robot package.  
The file has to have one argument per line. 

For example, the `fido.urdf.xacro` from the `fido_description` package might have the two arguments `simulation` and `sensors` to decide whether to include links/joints specific for simulation and  perception, repsectively. These can be set in the   `fido_robcogen/config/xacro_args.txt` file as follows:
```
simulation:=false
sensors:=true
```
Then, the filename would be passed as a fourth argument to the `robcogen` macro:
```
cmake_minimum_required(VERSION 3.10.2)
project(fido_robcogen)

...

robcogen(fido fido_description fido ${CMAKE_CURRENT_SOURCE_DIR}/config/xacro_args.txt)

...
```

## Limitations
- The transform files assumes the feet are named `LF_FOOT`, `RF_FOOT`, `LH_FOOT` and `RH_FOOT`.
  If you have different names for your end effectors, you have to manually change them [here](config/robot.dtdsl).
- The robot is assumed to have an IMU with a link called `imu_link` acting as coordinate frame for its measurementas.  
  Again, you can change the name in the template [here](config/robot.dtdsl).
- The xacro file path is assumed to be `${ROBOT_DESCRIPTION_PKG_NAME}/urdf/${ROBOT_XACRO_NAME}.xacro.urdf`
- links and joints names inside the xacro file **CANNOT**  start with a number
- The name of the robot defined the xacro file has to be `${ROBOT_NAME}`

## Credits
**Main author:** Marco Camurri 
All the material under the `external` folder (i.e., the `RobCoGen`  and `urdf2kindsl` programs) have been originally developed by Marco Frigerio. 

## License
This repository is released under the BSD License. See the LICENSE file for more details.
