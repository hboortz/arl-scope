#!/usr/bin/env bash


source file_utils.sh


download_ar_tools() {
    if [ ! -d "ar_tools" ]; then
        git clone git@github.com:arlolinscope/ar_tools.git
    fi
}


catkin_src
download_ar_tools
cd ar_tools
rosmake
