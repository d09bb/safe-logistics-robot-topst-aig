#!/bin/bash

# PC <-> TOPST network
export PC_TOPST_IP=${PC_TOPST_IP:-192.168.50.10}
export TOPST_IP=${TOPST_IP:-192.168.50.20}

# PC <-> Raspberry Pi Wi-Fi network
export PI_IP=${PI_IP:-192.168.0.24}

# PC address used by Raspberry Pi when sending perception
export PC_IP=${PC_IP:-192.168.0.8}

# Raspberry Pi <-> AI-G wired network
export PI_AIG_IP=${PI_AIG_IP:-192.168.60.1}
export AIG_IP=${AIG_IP:-192.168.60.2}

# UDP/TCP ports
export PERCEPTION_PORT=${PERCEPTION_PORT:-6002}
export TOPST_PORT=${TOPST_PORT:-5005}
export CMD_PORT=${CMD_PORT:-5006}
export AIG_FRAME_PORT=${AIG_FRAME_PORT:-7000}

# Ultrasonic sensor HC-SR04 GPIO pins, BCM numbering
export ULTRASONIC_TRIG=${ULTRASONIC_TRIG:-22}
export ULTRASONIC_ECHO=${ULTRASONIC_ECHO:-25}

# Ultrasonic threshold
export ULTRA_STALE_MS=${ULTRA_STALE_MS:-500}

# ArUco filtering

# ArUco distance / stability tuning

# Ultrasonic obstacle threshold
# 130mm = 13cm

# Ultrasonic obstacle threshold
# 130mm = 13cm
export ULTRA_OBSTACLE_MM=130

# ArUco far-distance tuning
export MIN_ARUCO_AREA=0
export ARUCO_STABLE_COUNT=1

# ArUco ID-specific filter tuning
export ARUCO_STABLE_COUNT=1
export MIN_ARUCO_AREA_ID0=0
export MIN_ARUCO_AREA_ID1=1800
export MIN_ARUCO_AREA_ID2=2500
