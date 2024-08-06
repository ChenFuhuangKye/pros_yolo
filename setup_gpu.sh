#!/bin/bash

docker run -it -p 9090:9090 --rm --gpus all -v "$(pwd)/src:/workspaces/src" ghcr.io/otischung/pros_ai_image:1.3.5-cuda /bin/bash