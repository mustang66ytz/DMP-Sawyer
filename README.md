
# Dynamic Movement Primitives on Sawyer 

This project implemented the DMP algorithms onto the Sawyer robot in the ROS framwork

## Getting Started

The DMP is a method in generating human-like nonlinear and high-dimensional movements, referenced from the paper below for the algorithm implementation
(http://www-clmc.usc.edu/publications/P/pastor-ICRA2009.pdf)

### Prerequisites

The project is implemented on ubuntu 16.04 with ROS kinetics
1.Create a new ros workspace
2.Git clone the repository into the newly created workspace
3.Build the ros workspace

## Running the tests

1.Run the dmp server for the calculation
```
$ roslaunch dmp_sawyer dmp.launch 
```
2.Run the data recorder to record robot joint space data, when finished, ctrl+c to exit, ignore errors
```
$ rosrun dmp_sawyer joints_recorder.py
```
3.Specify the new initial joint state and goal joint state in the dmpMassage.py script
4.Run the dmpMassage.py to generate DMP plan and move the robot arm accordingly
```
$ rosrun dmp_sawyer dmpMassage.py
```

## Built With

* [rospy](http://wiki.ros.org/rospy) - The robotics framework used
* [Sawyer](https://www.rethinkrobotics.com/sawyer/) - Robot hardware

## Authors

* **Taozheng Yang** - *Initial work* - 

