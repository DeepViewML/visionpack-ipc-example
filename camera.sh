#!/bin/sh

DEVICE=/dev/video3
WIDTH=1920
HEIGHT=1080

gst-launch-1.0 v4l2src device=${DEVICE} ! \
    video/x-raw,width=${WIDTH},height=${HEIGHT} ! \
    vslsink path=/tmp/camera.vsl
