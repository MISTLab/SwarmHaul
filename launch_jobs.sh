#!/bin/bash

set -e
# ./make_release.sh

# Experiments for Collobrative transoprt 

# Launch experiments
#


# SCRIPT=Object_movement_test
ROBOT_RANGE=(25 50 100)
PATH_ENUM=(straight zigzac straight_rot)
INTER_CAGING_DIST=(0.45 0.65 0.85)
MASS=5

# Submit jobs 

for SEED in $(seq 1 30)
do
    for ((IDX=0; IDX<${#PATH_ENUM[*]}; IDX++));
    do
        PATH="${PATH_ENUM[IDX]}"
        
        for ROBOTS in 25 50 100 
        do
            
            for INTER_CAGE_DIST in 0.45 0.65 0.85
            do
                
                RUNID=${ROBOTS}_${PATH}_${INTER_CAGE_DIST}_${MASS}_${SEED}
                OUTFILE=${RUNID}.csv
                POSFILE=pos_${RUNID}.csv
                # echo "Descriptive RUNID: $ROBOTS $PATH $INTER_CAGE_DIST $MASS $SEED" 
                echo "RUNID: $RUNID" 
                sbatch --job-name=${RUNID} run_job.sh $ROBOTS $PATH $INTER_CAGE_DIST $MASS $SEED
                sleep 1
            done
        done
    done
done
