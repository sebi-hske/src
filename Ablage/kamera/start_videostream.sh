#! /bin/bash

# change receivers ip 10.42.0.206
# wenn device nicht gefunden, mit ls ~/dev/ o.ä. alle devices anzeigen lassen
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! x264enc ! rtph264pay ! udpsink host=172.18.174.218  port=5600
