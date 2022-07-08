#!/bin/zsh
# Clear ports 8000 and 9090;
roslaunch rosbridge_server rosbridge_websocket.launch & sleep 2;
firefox localhost:8000/hmi_buttons.html;