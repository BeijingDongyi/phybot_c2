#!/bin/bash

# export LD_LIBRARY_PATH=./lib:./plugins:./platforms:./lcm/release/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=../dev/lib:../dev/platforms:./lcm/release/lib:$LD_LIBRARY_PATH

./application_systemd.*
echo "程序启动"

