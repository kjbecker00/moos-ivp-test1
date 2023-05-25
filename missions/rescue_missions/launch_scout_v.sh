#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_scout_v.sh    
#  Mission: rescue_baseline
#   Author: Ray Turrisi 
#   LastEd: May 11

./launch_vehicle \
--shore=192.168.1.79 \
--ip=$(ben_b) \
--vname=ben \
--vrole=scout \
--tmate=abe \
--speed="5" \
--maxspd="5"