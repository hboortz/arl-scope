#!/usr/bin/env bash


source file_utils.sh


download_arl_scope() {
    if [ ! -d "arl-scope" ]; then
        git clone git@github.com:hboortz/arl-scope.git
    fi
}

catkin_src
download_arl_scope
catkin_root
catkin_make
