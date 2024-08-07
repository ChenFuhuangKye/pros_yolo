FROM ocar1054/pros_yolo:latest

RUN rm -rf /workspaces/src/*
COPY ./src /workspaces/src

WORKDIR /workspaces

RUN pip install --no-cache-dir -r /workspaces/src/requirements.txt
CMD ["bash", "-l"]