#!/bin/bash

sudo cp ./arduino.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sleep 2
sudo service udev restart
echo "finish "
