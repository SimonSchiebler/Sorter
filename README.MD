# Sorter
This Repository contains code and documentation for a memory card sorting robot i built for a course at my university. 
The robot is capable of scanning a flat surface, detecting memory cards on it and sort them using a pneumatic grabber.
You can see the robot in action [here](https://www.youtube.com/watch?v=8ajy1oJylLM)

## Hardware
The main control components of the robot are a Raspberry-Pi and a BTT SKR 1.4 .
The SKR is a 3D printer board intended to drive the stepper motors and to control all physical components of the robot via its GPIO pins.
The advantage of using a 3D Printer board for this application is that it is very well documented and easy to control using G-Code sent over a USB connection. This way we can use the Marlin funtions for endstop-detection, homing, coordinate translation etc. by calling them with G-Code commands.
The Raspberry-Pi is there to run all the main logic like Image-Processing and Coordination of the Sled. Also it facilitates access to the system by exposing SSH via Wifi and providing the user with a touchscreen interface.

Further a small vacuum pump, a pneumatic piston and a solenoid valve enable for picking and placing of the cards.
The pneumatic system is powered by an external airbrush comressor.

The camera is a 5MP Waveshare Raspberry Pi camera mounted directly on the sled. For illumination a household led strip is used that is hooked up in series with a potentiometer to the power source.

## Software
The whole system uses the Robot operating system ([ROS Melodic](http://wiki.ros.org/melodic)).
ROS enables you to separate the robot logic into so called nodes that run as separate processes and communicate with each other over the network using predefined messages. This helps to keep the codebase clean and modular.

The main logic is separated into **control** and **analyse**.
All nodes in the [Control](./src/rosSorter/src/control/) folder serve to physically control the robot. They control the sled and the grabber.

[Analyse](./src/rosSorter/src/analyse/) contains nodes to scan the field and to extract the cards using openCV.

[SolveMemory.py](./src/rosSorter/src/SolveMemory.py) is the main node. It provides the **solve Memory** service that when called triggers the whole process of scanning and solving.
The image matching logic is also situated here.

### Dependencies
[ROS Melodic](http://wiki.ros.org/melodic)

[openCV contrib 3.4.2.16](https://pypi.org/project/opencv-python/3.4.2.16/)
this version is necessary because newer versions of openCV don't provide access to the SIFT algorithm.

[raspicam node](https://github.com/UbiquityRobotics/raspicam_node) This is an ROS node that provides access to the Raspberry pi camera.
