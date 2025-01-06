WLAN-Verbindung mit Roboter:
    ssh ubuntu@10.42.0.1
        passwort: ipek2023

Node auf Roboter starten:
    ros2 launch turtlebot3_bringup robot.launch.py
Neues Terminal auf Roboter starten:
    python3 ArucoDistance.py

Neues Terminal
    In workspace navigieren und action server starten
        cd ~/ros2_ws
        colcon build (optional)
        source install/setup.bash
        ros2 run ar_pipe_server  PipeServer

Neues Terminal
    source install/setup.bash
        Goal zum bewegen senden:
            ros2 action send_goal /velocity ar_interace/action/Mode velocity:\ 0.0\