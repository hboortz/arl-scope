#!/usr/bin/env bash

cd ~/Downloads
wget http://ardupilot.com/wp-content/plugins/download-monitor/download.php?id=111 -O apmplanner2.deb
sudo dpkg -i apmplanner2.deb
sudo apt-get -f install
sudo dpkg -i apmplanner2.deb
rm -f apmplanner2.deb
