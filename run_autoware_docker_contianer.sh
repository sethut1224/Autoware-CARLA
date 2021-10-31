#!/bin/bash

if [ "$#" != "1" ]; then
          echo "Usage: $0 [image_tag]"
          exit 1
fi

TAG=$1
IMAGE=autoware/autoware:${TAG}

RUNTIME="--runtime=nvidia"

USER_ID=1000

echo "Launching $IMAGE"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

SHARED_DOCKER_DIR=/home/autoware/shared_dir
SHARED_HOST_DIR=$HOME/shared_dir

mkdir -p $SHARED_HOST_DIR

VOLUMES="--volume=$XSOCK:$XSOCK:ro
         --volume=$XAUTH:$XAUTH:ro
         --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
         --volume=/etc/localtime:/etc/localtime:ro"

docker run \
    -it \
    $VOLUMES \
    $DEVICES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="USER_ID=$USER_ID" \
    --privileged \
    --gpus all \
    --net=host \
    $IMAGE
