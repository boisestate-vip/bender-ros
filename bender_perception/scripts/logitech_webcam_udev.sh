#!/bin/bash

echo 'KERNEL=="video*", SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0843", SYMLINK+="videoLogitech"' | sudo tee 10-logitech-webcam.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
