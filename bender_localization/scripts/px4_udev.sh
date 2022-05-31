#!/bin/bash

echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5740", SYMLINK+="ttyPixhawk", RUN+="/bin/stty -F /dev/ttyPixhawk 115200 raw -echo"' | sudo tee 10-pixhawk-px4.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
