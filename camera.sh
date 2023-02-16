#!/bin/sh

DEVICE=/dev/video2
WIDTH=3840
HEIGHT=2160

gst-launch-1.0 v4l2src device=${DEVICE} ! \
    video/x-raw,width=${WIDTH},height=${HEIGHT} ! \
    vslsink path=/tmp/camera.vsl
