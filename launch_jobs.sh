#!/bin/bash

set -e
# ./make_release.sh

# Experiments for Collobrative transoprt 

# Launch experiments
#


# SCRIPT=Object_movement_test
# ROBOT_RANGE=(25 50 100)
# PATH_ENUM=(straight zigzac straight_rot)
# INTER_CAGING_DIST=(0.45 0.65 0.85)
MASS=5
OBJECT_SHAPE=0
ROBOT_PATH_ENUM=0

# Submit jobs for transport

for SEED in $(seq 1 30)
do
    for ROBOT_PATH in "straight" "zigzac" "straight_rot"
    do
        
        for ROBOTS in 25 50 100 
        do
            
            for INTER_CAGE_DIST in 0.45 #0.65 0.85
            do
                if [[ ROBOTS -eq 50 ]]
                then
                    OBJECT_SHAPE=1

                elif [[ ROBOTS -eq 100 ]]
                then
                    OBJECT_SHAPE=2
                else
                    OBJECT_SHAPE=0
                fi
		if [[ "$ROBOT_PATH" == "straight" ]]
                then
                    ROBOT_PATH_ENUM=1

                elif [[ "$ROBOT_PATH" == "zigzac" ]]
                then
                    ROBOT_PATH_ENUM=2
                else
                    ROBOT_PATH_ENUM=3
                fi
                RUNID="${ROBOTS}_${ROBOT_PATH_ENUM}_${SEED}"
                echo "RUNID:${RUNID}" 
                echo "$ROBOTS $ROBOT_PATH $INTER_CAGE_DIST $MASS $OBJECT_SHAPE $SEED"
	        sbatch --job-name=${RUNID} run_job.sh $ROBOTS $ROBOT_PATH $INTER_CAGE_DIST $MASS $OBJECT_SHAPE $SEED
                sleep 2
            done
        done
    done
done

