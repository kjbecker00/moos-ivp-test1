#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: launch_sim.sh    
#  Mission: rescue_baseline
#   Author: Ray Turrisi 
#   LastEd: May 11

dir="swim_files"

# Check for the optional argument and filter files accordingly
if [[ $# -eq 1 ]]; then
    case "$1" in
        x)
            swim_files=($(find "$dir" -type f -name "*x*.txt"))
            ;;
        g)
            swim_files=($(find "$dir" -type f -name "*g*.txt"))
            ;;
        *)
            echo "Invalid argument. Use 'x' or 'g' or pass no argument."
            exit 1
            ;;
    esac
else
    swim_files=("$dir"/*)
fi

num_files=${#swim_files[@]}

if [[ $num_files -eq 0 ]]; then
    echo "No files found with the given filter."
    exit 1
fi

rand_idx=$((RANDOM % num_files))

swim_file="${swim_files[rand_idx]}"
printf "Swimming with $swim_file\n"

./launch_alt.sh 10 \
--rs2 \
--swim_file=${swim_file}
