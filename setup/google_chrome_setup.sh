#!/usr/bin/env bash

cd ~/Downloads
if [ ! -f ~/Downloads/google-chrome-stable_current_amd64.deb ]; then
    wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
fi
sudo dpkg -i google-chrome-stable_current_amd64.deb
sudo apt-get update
sudo apt-get upgrade
sudo apt-get -f install
sudo dpkg -i google-chrome-stable_current_amd64.deb
