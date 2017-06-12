#!/bin/bash

source /home/docker/wv/install/setup.bash
bridge-app &
roscore -p 8001
