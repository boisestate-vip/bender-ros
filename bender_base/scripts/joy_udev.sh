#!/bin/bash

sudo sh -c 'echo "KERNEL==\"js?\", MODE==\"0666\", SYMLINK+=\"js_f710\"" > /etc/udev/rules.d/10-all-js-f710.rules'
sudo chmod +x /etc/udev/rules.d/10-all-js-f710.rules
sudo service udev restart
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
exit
$SHELL