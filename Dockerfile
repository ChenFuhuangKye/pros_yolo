FROM ghcr.io/otischung/pros_ai_image:latest

RUN rm -rf /workspaces/src/*
COPY ./src /workspaces/src

WORKDIR /workspaces

RUN pip install --no-cache-dir -r /workspaces/src/requirements.txt
RUN colcon build 
CMD ["bash", "-l"]