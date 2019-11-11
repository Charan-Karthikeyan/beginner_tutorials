[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
## Publisher and Subscriber ROS cpp

# Overview

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

#TF Frames
The talker.cpp file has been changed to bradcast the static tf frames called talk with respect to the world in the frames.

The whole process is controlled using ros time and is time stamped. The frames can be visualized using the following command 
in a terminal
```
rosrun rqt_tf_tree rqt_tf_tree
```
To look at the values through the echo module we have to run the following command in a new terminal
```
rosrun tf tf_echo /world /talk
```
To generate and view the frames we use the following command
```
rosrun tf view_frames
```
This command generates a pdf and agviz document which can be viewed in the folder that you run in the folder.
The output of the commands are shown in the image below
</p>
<p align="center">
<img src="/images/tf_final.png">
</p>
</p>

#Running the Unit Tests
The unit tests are written in the test folder and can be launched using the launch file in the test folder.
To run the unit test we have to run the following commands
```
cd ~/catkin_ws
catkin_make
rostest beginner_tutorials test.launch
```
This will show the following output

</p>
<p align="center">
<img src="/images/unit_test.png">
</p>
</p>

#Rosbag record and play 
The rosbag has been used to read and save all the outputs from the nodes when they run.
The record the values we have to run the following command
The command should be run in the folder that the rosbag file is to be created.
```
rosbag record --duration=10 -a -O rostopicsRecord.bag
```
This saves the code for 10 seconds and writes it to a rosbag file.
The saved files can then be run by using 
```
rosrun beginner_tutorials listner
rosbag play rostopicsRecord.bag
```
The output will be as shown below
</p>
<p align="center">
<img src="/images/rosbag_out.png">
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

#Add tag to the branch

Run this command inside the beginner_tutotials to create tag for the branch
```
git tag -a Week11_HW_Release -m "week11_hw tag"
```
to check the tags. Type in 
```
git tag
```
this will show the tag of the branch.

#Merge with Master

To merge with the master. We have to shift to the master by typing in 
```
git checkout master
```
Then type
```
git merge origin Week11_HW
```
This will merge with the master branch. If any conflicts arise it will automatically resolve them. If not they 
have to be manually rectified and pushed.






