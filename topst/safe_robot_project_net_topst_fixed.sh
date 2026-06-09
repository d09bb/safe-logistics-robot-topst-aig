#!/bin/sh

PC_IP="192.168.50.10"
TOPST_IP="192.168.50.20"

echo "[TOPST_NET] set eth0 only once"

ifconfig eth0 $TOPST_IP netmask 255.255.255.0 up

# 중복 route 제거
route del -net 192.168.50.0 netmask 255.255.255.0 eth0 2>/dev/null || true
route del -net 192.168.50.0 netmask 255.255.255.0 eth0 2>/dev/null || true
route del -net 192.168.50.0 netmask 255.255.255.0 eth0 2>/dev/null || true

route add -net 192.168.50.0 netmask 255.255.255.0 eth0 2>/dev/null || true

echo "[TOPST_NET] result:"
ifconfig eth0
route -n

echo "[TOPST_NET] ping PC"
ping -c 2 $PC_IP
