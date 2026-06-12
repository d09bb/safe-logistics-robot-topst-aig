# ===== SAFE ROBOT NETWORK ENV =====

# PC <-> Raspberry Pi Wi-Fi network
export PI_IP=192.168.0.12

# Pi sends perception relay to PC Wi-Fi IP
export PC_IP=192.168.0.60

# Raspberry Pi <-> AI-G wired network
export PI_AIG_IP=192.168.60.1
export AIG_IP=192.168.60.2

# PC <-> TOPST wired network
export PC_TOPST_IP=192.168.50.10
export TOPST_IP=192.168.50.20

# UDP/TCP ports
export PERCEPTION_PORT=6002
export TOPST_PORT=5005
export CMD_PORT=5006
export AIG_FRAME_PORT=7000

# Ultrasonic sensor
export ULTRASONIC_TRIG=22
export ULTRASONIC_ECHO=25
export ULTRA_STALE_MS=500
export ULTRA_OBSTACLE_MM=250

# ArUco filter
export MIN_ARUCO_AREA=0
export ARUCO_STABLE_COUNT=1
export MIN_ARUCO_AREA_ID0=0
export MIN_ARUCO_AREA_ID1=1800
export MIN_ARUCO_AREA_ID2=2500
