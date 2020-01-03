#!/bin/bash
[ $(id -u) != "0" ] && { echo "Error: You must be root to run this script"; exit 1; }
cd $(dirname $0)

cp roscore-daemon.sh /opt/ros/
chmod +x /opt/ros/roscore-daemon.sh

cp roscore /etc/init.d/
chmod +x /etc/init.d/roscore

cp roscore.service /lib/systemd/system/

systemctl daemon-reload
systemctl enable roscore
systemctl start roscore
