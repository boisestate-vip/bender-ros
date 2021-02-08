# bender_firmware
This is a PlatformIO project that contains the firmware code for the Teensy 3.6 MCU. An easy way to get started with PlatformIO is to use Visual Studio Code, install the `PlatformIO IDE` extension, and just simply add this directory as an existing project. 

This project requires `rosserial_arduino` to generate the necessary header files to be included under the `lib` directory. These files are excluded from the git repository due to the number of files involved. To generate the required header files, first make sure you are in the `bender_firmware` directory then execute
```
sudo apt install ros-$ROS_DISTRO-rosserial-arduino && rosrun rosserial_arduino make_libraries.py lib/
```

## Resources for setting up `hardware_interface::RobotHW` in general
- https://roscon.ros.org/2014/wp-content/uploads/2014/07/ros_control_an_overview.pdf
- https://answers.ros.org/question/356894/help-to-understand-how-to-implement-diff_drive_controller/

## Resources for integrating `rosserial` with `ros_control` hardware interface
- https://vimeopro.com/osrfoundation/roscon-2014/video/106992629
- https://roscon.ros.org/2014/wp-content/uploads/2014/07/serious-rosserial.pdf
- https://github.com/jackal/jackal_robot/tree/melodic-devel/jackal_base

