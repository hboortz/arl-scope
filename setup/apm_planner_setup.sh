#!/usr/bin/env bash

cd ~/Downloads
if [! -f ~/Download/apmplanner2.deb ]; then
    wget http://ardupilot.com/wp-content/plugins/download-monitor/download.php?id=111 -O apmplanner2.deb
fi

sudo dpkg -i apmplanner2.deb
sudo apt-get -f install
sudo dpkg -i apmplanner2.deb
rm -f apmplanner2.deb
