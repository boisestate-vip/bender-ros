#!/bin/bash

echo 'KERNEL=="video*", SUBSYSTEM=="video4linux", ATTRS{index}=="0", ATTR{name}=="Logitech Webcam C930e", SYMLINK+="videoLogitech"' | sudo tee 10-logitech-webcam.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
