#!/bin/bash

cd /opt/PX4-Autopilot || exit 1

# Drone 1: avvia anche il server Gazebo
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1 &

sleep 5

# Droni successivi
for i in {2..10}
do
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,$((i-1))" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i $i &
sleep 2
done

wait