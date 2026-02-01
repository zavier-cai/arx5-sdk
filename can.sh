#!/bin/bash

CAN_INTERFACE="can0"

# 检查设备是否存在
if ! ip link show "$CAN_INTERFACE" > /dev/null 2>&1; then
    echo "Error: $CAN_INTERFACE not found, please check if canable device is connected"
    exit 1
fi

# 配置和启动
sudo ip link set $CAN_INTERFACE down 2>/dev/null
sudo ip link set $CAN_INTERFACE type can bitrate 1000000
sudo ip link set $CAN_INTERFACE txqueuelen 1000
sudo ip link set $CAN_INTERFACE up

echo "$CAN_INTERFACE started successfully"
