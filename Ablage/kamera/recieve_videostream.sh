
# ip aus wlan optionen ipek2

gst-launch-1.0 udpsrc address=10.42.0.206 port=5600 ! application/x-rtp, encoding-name=H264 ! rtph264depay ! avdec_h264 ! autovideosink