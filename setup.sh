#!/bin/bash

docker run -it -p 9090:9090 --rm   -v "$(pwd)/src:/workspaces/src" ghcr.io/otischung/pros_ai_image:latest /bin/bash