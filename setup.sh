#!/bin/bash

docker run -it -p 9090:9090 -v"$(pwd)/src:/workspaces/src" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  kyehuang/pros_yolo:latest  /bin/bash