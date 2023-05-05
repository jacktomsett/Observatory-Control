#!/bin/bash

#Basic sequence capture script.

N=60
EXPOSURE=4.5s
for ((i=1;i<=N;i++));
do
    gphoto2 --set-config capturetarget=card --set-config=/main/actions/bulb=1 --wait-event=$EXPOSURE
done
