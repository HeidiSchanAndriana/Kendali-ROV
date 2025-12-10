#!/usr/bin/env bash
pkill -f sim_vehicle.py 2>/dev/null || true
pkill -f mavproxy.py 2>/dev/null || true
pkill -f mavros_node 2>/dev/null || true
pkill -f QGroundControl 2>/dev/null || true
echo 'Stopped.'
