[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# ROS Beginners Tutorial

## Overview
ROS Packager for consisting of publisher and subscriber node for a custom string

## Standard install via command-line
```
cd /home/user/catkin_ws/src
git clone --recursive https://github.com/Sameep2808/beginner_tutorials.git
cd ..
catkin_make clean && catkin_make
```

## Dependencies

- Ubuntu 20.04.3 
- ROS Neotic
- python (if not already installed)
- catkin_pkg (if not already installed while installing ROS)
For installing catkin_pkg and directly adding it to PYTHONPATH.
```
pip install catkin_pkg
```
Check if the catkin_pkg path is added to PYTHONPATH by using the following
command
```
echo $PYTHONPATH
```

## Installation

ROS Installation:
Install the ROS Kinetic for Ubuntu and it's dependencies using the [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)


## Run
Make sure that a roscore is up and running: 
```
roscore
```

Make sure you have sourced your workspace's setup.sh file after calling catkin_make but before trying to use your applications:
```
cd ~/catkin_ws
source ./devel/setup.bash
```

To Run the Publisher Node
```
rosrun beginner_tutorials talker 
```

To Run the listener Node
```
rosrun beginner_tutorials listener
```

To call the service.(Open a new terminal)
```
rosservice call changeString "<any new string the user wants to enter>"
```

# Run Using the launch file
We can directly use our new launch file
```
roslaunch beginner_tutorials changeString.launch frequency:=<integer value for frequency loop>
```
The frequency parameter is optional.

To call the service.(Open a new terminal)
```
rosservice call changeString "<any new string the user wants to enter>"
```

# Inspecting  TF frames
```
roslaunch beginner_tutorials changeString.launch
```
In a new Terminal window
```
rosrun tf2_tools view_frames.py
evince frames.pdf
```
Using rqt_tf_tree
```
rosrun rqt_tf_tree rqt_tf_tree
```
Using tf_echo
```
rosrun tf tf_echo world talk
```

# Running ros test
```
catkin_make run_tests_beginner_tutorials
```

# Recording bag files with the launch file
```
roslaunch beginner_tutorials changeString.launch frequency:=<integer value for frequency loop> run_rosbag:=true
```

# Disable bag file recording
```
roslaunch beginner_tutorials changeString.launch frequency:=<integer value for frequency loop> run_rosbag:=false
```

# Inspecting the bag file and playing back the bag file with the Listener node demonstration
Start roscore
```
roscore
```
Run the listener Node (Another terminal) 
```
rosrun beginner_tutorials listener
```
Run the bag file
```
rosbag play -r 2 <your bagfile>
```

## Working with Eclipse IDE ##

## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)
```
mkdir -p ~/workspace
cd ~/workspace
git clone --recursive https://github.com/Sameep2808/beginner_tutorials.git
```

In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of cpp-boilerplate

```
cd ~/workspace
mkdir -p boilerplate-eclipse
cd boilerplate-eclipse
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 /home/user/catkin_ws/src/beginner_tutorials
```

## Import

Open Eclipse, go to File -> Import -> General -> Existing Projects into Workspace -> 
Select "boilerplate-eclipse" directory created previously as root directory -> Finish

# Edit

Source files may be edited under the "[Source Directory]" label in the Project Explorer.


## Build

To build the project, in Eclipse, unfold boilerplate-eclipse project in Project Explorer,
unfold Build Targets, double click on "all" to build all projects.


## Debug


1. Set breakpoint in source file (i.e. double click in the left margin on the line you want 
the program to break).

2. In Eclipse, right click on the boilerplate-eclipse in Project Explorer, select Debug As -> 
Local C/C++ Application, choose the binaries to run (e.g. shell-app).

3. If prompt to "Confirm Perspective Switch", select yes.

4. Program will break at the breakpoint you set.

5. Press Step Into (F5), Step Over (F6), Step Return (F7) to step/debug your program.

6. Right click on the variable in editor to add watch expression to watch the variable in 
debugger window.

7. Press Terminate icon to terminate debugging and press C/C++ icon to switch back to C/C++ 
perspetive view (or Windows->Perspective->Open Perspective->C/C++).


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project. If you're interested in this, try it out yourself and contact me on Canvas.
