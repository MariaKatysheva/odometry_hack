#!/usr/bin/env bash
# Проверка решения как у жюри: сборка, нода + судья (hackathon_solution_checker), ros2 bag play.
#   docker run --rm -v <ros2_ws>:/src_ws:ro -v <check-code>:/check:ro -v <bag>:/bag:ro ros:humble-ros-base bash /src_ws/test_in_docker.sh [rate]
set -e
RATE="${1:-1}"
source /opt/ros/humble/setup.bash
mkdir -p /ws/src && cp -r /src_ws/src/* /ws/src/ && cp -r /check/src/checker_ros /ws/src/
cd /ws && colcon build --packages-select tram_vehicle_msgs tram_backup_odometry hackathon_solution_checker 2>&1 | tail -4
source install/setup.bash
ros2 launch tram_backup_odometry odometry.launch.py > /tmp/node.log 2>&1 &
NODE=$!
ros2 run hackathon_solution_checker metrics --ros-args -p report_period_sec:=2.0 > /tmp/metrics.log 2>&1 &
MET=$!
sleep 5
ros2 bag play /bag --rate "$RATE" > /tmp/play.log 2>&1
sleep 8
kill -INT $MET; sleep 3; kill -INT $NODE; sleep 1
echo "=== нода"; grep -v "^$" /tmp/node.log | head -20
echo "=== судья (последний отчёт после конца записи)"; grep "Velocity metrics" /tmp/metrics.log | tail -1; grep "Position metrics" /tmp/metrics.log | tail -1
