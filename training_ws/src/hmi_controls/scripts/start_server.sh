#!/bin/bash
# Clear ports 8000 and 9090;
fuser -k 8000/tcp;
fuser -k 9090/tcp;
serverPath=`rosrun hmi_controls get_package_path.py`;
echo $serverPath;
cd "$serverPath";
python -m SimpleHTTPServer;