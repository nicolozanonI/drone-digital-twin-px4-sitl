#!/bin/bash

cd /opt/PX4-Autopilot || exit 1

PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1 &

sleep 5

for i in {1..10}
do
(
  echo "param set NAV_DLL_ACT 0"
  echo "param set NAV_RCL_ACT 0"
  echo "param save"
  sleep inf        # mantiene stdin aperto
) | env PX4_GZ_STANDALONE=$([ $i -gt 1 ] && echo 1) \
     PX4_SYS_AUTOSTART=4001 \
     PX4_GZ_MODEL_POSE="0,$((i-1))" \
     PX4_SIM_MODEL=gz_x500 \
     ./build/px4_sitl_default/bin/px4 -i $i &
done

wait