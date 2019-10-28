## Publisher and Subscriber ROS cpp

# Overview

This a basic introduction to the usage of ROS nodes with the Publisher and the Subscriber. 
The publisher and the subscriber have also been modified to C++11 standards and checked using the cppcheck and the cpplint.

#Build and Install the package
```
cd
#Create your workspace
mkdir -p catkin_ws
cd catkin_ws
#Create source folder inside the workspace
mkdir -p src
#Initialize the workspace
catkin_make
cd src
#Clone the Package into the source folder
git clone https://github.com/Charan-Karthikeyan/beginner_tutorials.git
cd ..
#Building the workspace
catkin_make
```

# Executiong the Package

Source the workspace
```
~ ./catkin_ws/devel/setup.bash
```

Open a new terminal tab and run roscore
```
roscore
```
Go back to the terminal we setup the workspace
```
rosrun beginner_tutorials talker
```
Open a new tab and runther to source the workspace then run the command
```
rosun beginner_tutorials listner
```
# Results from cpplint and cppcheck
To get the output from the cppcheck
```
cd beginner_tutorials
 cppcheck --enable=all --std=c++11 -I ../../devel/include/ -I ../../../../../../../opt/ros/kinetic/include/ -I ../../../../../../../usr/include/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```
To get the output from the cpplint 
```
cd beginner_tutorials
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

```
The results from the checks have been added as a text file.
  
# Dependencies and Assumptions
ROS Kinetic is installed in the computer and working correctly and there is no other package with the same name in the workspace.





