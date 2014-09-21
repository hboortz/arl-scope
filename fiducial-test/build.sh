#!/usr/bin/env bash
g++ src/main.cc `pkg-config --libs opencv` -o fiducial_test
