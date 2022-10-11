#!/bin/bash

# ----- Has to be run from the folder 'laser_trace'

g++ laser_trace.cpp -fPIC -O3  -msse4 -shared -I/usr/include/python3.8 -I/usr/lib/python3.8/site-packages/numpy/core/include -l:libboost_python310.a -lpython3.8 -o laser_trace.so

mv laser_trace.so ../pf_localisation/

