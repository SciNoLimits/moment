#!/usr/bin/env bash
# Spawn a red, green and blue 3 cm cube into the running Gazebo world for testing.
# Usage: bash spawn_cubes.sh [x] [y]     (centre of the row, metres in the world frame; default 0.5 0)
# Positions are printed so you can compare them with what the detector returns.
X=${1:-0.5}
Y=${2:-0.0}
S=0.03
Z=$(python3 -c "print($S/2)")

spawn () {   # name r g b dy
  local y=$(python3 -c "print($Y + $5)")
  local sdf="<sdf version='1.9'><model name='$1'><link name='link'>
    <inertial><mass>0.02</mass><inertia><ixx>3e-6</ixx><iyy>3e-6</iyy><izz>3e-6</izz></inertia></inertial>
    <collision name='c'><geometry><box><size>$S $S $S</size></box></geometry>
      <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
    <visual name='v'><geometry><box><size>$S $S $S</size></box></geometry>
      <material><ambient>$2 $3 $4 1</ambient><diffuse>$2 $3 $4 1</diffuse></material></visual>
  </link></model></sdf>"
  ros2 run ros_gz_sim create -name "$1" -x "$X" -y "$y" -z "$Z" -string "$sdf"
  echo "spawned $1 at world ($X, $y, $Z)"
}

spawn red_cube   1 0 0  0.08
spawn green_cube 0 1 0  0.00
spawn blue_cube  0 0 1 -0.08
