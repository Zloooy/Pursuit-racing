#!/usr/bin/bash
#follower_speed:=${1} follower_reaction_frequency:=${2}
echo ${@:1}
source devel/setup.bash
roslaunch pursuit_racing pursuit_racing.launch ${@:1}