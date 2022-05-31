#!/bin/bash

echo 'ACTION=="add|change", SUBSYSTEM=="net", KERNEL=="can*", ENV{INTERFACE}=="can*", ATTR{tx_queue_len}="1000", RUN+="/usr/bin/ip link set $name type can bitrate 250000", RUN+="/usr/bin/ip link set up $name"' | sudo tee /etc/udev/rules.d/71-can-interface.rules

