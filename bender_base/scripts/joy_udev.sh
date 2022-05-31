#!/bin/bash

echo 'KERNEL=="js*", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="0002", SYMLINK+="js_f710"' | sudo tee /etc/udev/rules.d/10-logitech-js-f710.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
