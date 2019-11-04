[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
## Publisher and Subscriber ROS cpp

#Overview

This a basic introduction to the usage of ROS nodes with the Publisher and the Subscriber. 
The publisher and the subscriber have also been modified to C++11 standards and checked using the cppcheck and the cpplint.

#License
The repository is licensed using BSD-3-Clause license given in this link [LICENSE](https://choosealicense.com/licenses/bsd-3-clause/)

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

#Executiong the Package
Open a terminal and run the command
```
sudo gedit ~/.bashrc
```

Add the following lines to source the workspace
```
source ./catkin_ws/devel/setup.bash
```
Save the file and exit.Then open a terminal and run source the bashrc file
```
source ~/.bashrc
```
Open a new terminal tab and run roscore
```
roscore
```
Then in an unused terminal
```
roslaunch beginner_tutorials begineer_tutorials.launch frequency:=10
```
or
```
roslaunch beginner_tutorials begineer_tutorials.launch
```
This runs the talker and subscriber at default rate of 20Hz
The talker and listner are launched in different terminals.
In another terminal run the command to change the message in the talker
```
rosservice call /customeMsg "hello" 
```
This changes the output as follows
</p>
<p align="center">
<img src="/images/out.png">
</p>
</p>

#Results from cpplint and cppcheck
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

#ROS funtions
To see the node graph run the following commanding during execution of the package
```
rqt_graph
```
To see the message logging in real-time run the following command
```
rqt_console
```

#Dependencies and Assumptions
ROS Kinetic is installed in the computer and working correctly and there is no other package with the same name in the workspace.





