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

# Submit jobs for transport

# Submit jobs for cageing
for SEED in $(seq 1 30)
do

    for OBJECT_SHAPE in 3 4 5 
    do
        PATH="none"

        for ROBOTS in 30 
        do
            for INTER_CAGE_DIST in 0.45 0.65 0.85
            do
                       
                RUNID=${ROBOTS}_${PATH}_${INTER_CAGE_DIST}_${MASS}_${OBJECT_SHAPE}_${SEED}
                echo "RUNID: $RUNID" 
                # sbatch --job-name=${RUNID} run_job.sh $ROBOTS $PATH $INTER_CAGE_DIST $MASS $OBJECT_SHAPE $SEED
                # sleep 0.1
            done
        done
    done
done