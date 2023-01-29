# MK2 Robot Arm

This project is a 3 DOF robot arm constructed using 3D printing and powered by stepper motors. The arm is driven by a CNC shield on arduino uno and power supply, and is designed for precise and accurate positioning control.
In this project both forward and inverse kinematics were constructed to model the robot motion towards the desired postions.

## Installation and setup

To install the necessary software, follow these steps:

1. Install ROS Neotic.
2. Install Arduino IDE.
3. Install ROSserial package by running the following commands in your terminal:  
`sudo apt-get install ros-noetic-rosserial-arduino`  
`sudo apt-get install ros-noetic-rosserial`  
4. Run the roscore command:
`roscore`  
5. In the Arduino libraries folder, run the ROS node:  
`rosrun rosserial arduino make libraries.py .`

6. Install the following libraries:
- Peter Corke robotics toolbox library.
- Spatial Maths for Python.
- tkinter library for GUI.

You install them by running the following commands in your terminal:  
`pip install robotics-toolbox-python`  
`pip install spatialmath-python`  
`pip install tk`  

7. Clone the repository:  
`git clone git@github.com:youssiefanas/MK2.git `
8. Copy the MK2_py folder to your ROS workspace (e.g. catkin_ws).
9. In your workspace directory, run:
`catkin_make`.
10. Connect the Arduino board to your computer using a USB cable.
11. Upload the Arduino sketch (included in this repository) to the board.
12. Run the following commands in separate terminal windows:

`roscore`.
`rosrun MK2_py MK2_gui.py`.
`rosrun rosserial_arduino serial_node.py /dev/ttyACM0`.


## Robot Kinematics

The robot configuration was constructed using the Peter Corke robotics toolbox. The D-H parameters were solved according to the configuration. The robotics toolbox allows you to solve forward kinematics using the `fkine()` function and inverse kinematics using the `ikine_LM()` function. The workspace of the robot was also identified using the toolbox.


## GUI overview

The GUI for this project allows you to control the MK2 robot arm and make it reach a certain position (x, y, z) coordinates or by moving each joint independently. It also includes buttons to move the robot arm to its home position and control the gripper.

To use the GUI, follow these steps:

1. The GUI will appear when you run the ROS node.
2. Input x, y, z coordinates in the input boxes and press the "Calculate" button to move the robot arm to that position.
3. Use the sliders to move each joint independently.
4. Press the "Home" button to move the robot arm to its home position.
5. Press the "Gripper release" and "Gripper Close" buttons to control the gripper.

## Additional features

In addition to the features described above, this project also includes the following:

- The ability to solve the forward and inverse kinematics.
- The ability to simulate the robot configuration according to a given position.
- The ability to send Stepper motors readings to the Arduino to control the physical MK2 robot arm.
- The output display of joint angles and servo readings in the GUI.

## Hardware Design and components

- The MK2 robot arm hardware was designed using 3D printing. The stepper motors, CNC shield, and power supply were selected for their reliability and performance.
 - you can find the resources below.

## Future work

There are many possibilities for future improvements to this project. Some ideas might include:

- Adding depth camera sensors to the end effector to detect objects and their 3D positions.
- Implementing motion on a trajectory.

## Resources

This project would not have been possible without the following resources:

- The Peter Corke robotics toolbox library was used for this project: https://github.com/petercorke/robotics-toolbox .

- The 3D models and design files for the MK2 robot arm are available at: https://www.thingiverse.com/thing:2520572 .
- you can find the selected components in this link: https://www.hackster.io/yasaspeiris/mk2-plus-robot-arm-controller-458d55 .

# Acknowledgements

- Professor Mahmoud El-samanty.
- Professor Haitham El-husseiny.
- The MeArm pocket size manipulator project was used as inspiration for this project.
