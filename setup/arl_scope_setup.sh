#!/usr/bin/env bash


source file_utils.sh


download_arl_scope() {
    if [ ! -d "arl-scope" ]; then
        git clone https://github.com/arlolinscope/arl-scope
    fi
}

catkin_src
download_arl_scope
catkin_root
catkin_make
