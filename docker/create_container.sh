#!/usr/bin/env bash

xhost +local:root

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

docker run --privileged \
    -i -d --gpus all \
    --volume $XSOCK:$XSOCK:rw \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -e SSH_AUTH_SOCK=/ssh-agent \
    -v "$(readlink -f """$SSH_AUTH_SOCK""")":/ssh-agent \
    --env DISPLAY=$DISPLAY \
    --env XAUTHORITY=$XAUTH \
    --env QT_X11_NO_MITSHM=0 \
    --env QT_X11_NO_XRENDER=0 \
    --volume $XAUTH:$XAUTH:rw \
    --net host \
    --pid host \
    -it \
    --name="chargedup_container" \
    dropbears_chargedup
