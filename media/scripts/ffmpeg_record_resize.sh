#!/bin/bash

today=`date +%Y-%m-%d-%H%M%S`
dims=`xdpyinfo | grep dimensions | sed -r 's/^[^0-9]*([0-9]+x[0-9]+).*$/\1/'`
dimx=`xrandr --current | grep '*' | uniq | awk '{print $1}' | cut -d 'x' -f1`
dimy=`xrandr --current | grep '*' | uniq | awk '{print $1}' | cut -d 'x' -f2`

ffmpeg -video_size 2560x1440 -framerate 30 -f x11grab -i :1.0+$dimx,0 -t ${1:-20} tmp0.mp4
ffmpeg -i tmp0.mp4 -r 16 -filter:v "setpts=0.25*PTS" tmp1.mp4
ffmpeg -i tmp1.mp4 -vf scale=1280:-1 ../videos/$today.mp4
rm tmp*


