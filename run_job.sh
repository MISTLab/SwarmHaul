#!/bin/bash
#SBATCH --account=def-beltrame
#SBATCH --time=12:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8

# Stop on any error
set -e

# Basic variables
ME=$(whoami)
BASEWORKDIR=${SCRATCH}/run_dir
HOMEDIR=${SCRATCH}/collobrative_transport
DATADIR=${SCRATCH}/data
mkdir -p ${DATADIR}

# Other useful variables
#export ARGOS_PLUGIN_PATH=${HOMEDIR}/build
#export LD_LIBRARY_PATH=${ARGOS_PLUGIN_PATH}:$LD_LIBRARY_PATH


# Script parameters
ROBOTS=${1}
PATH=${2}
INTER_CAGE_DIST=${3}
MASS=${4}
RANDOMSEED=${5}


# Run id, used as base file name
RUNID=${ROBOTS}_${PATH}_${INTER_CAGE_DIST}_${MASS}_${RANDOMSEED}

echo "${RUNID}"

# Output file names and directories
WORKDIR=${BASEWORKDIR}/${ME}_${RUNID}
OUTFILE=${RUNID}.csv
EXPERIMENT=run_${RUNID}.argos

# Create directory
rm -rf ${WORKDIR}
mkdir -p ${WORKDIR}
mkdir -p ${WORKDIR}/buzz_scripts
mkdir -p ${WORKDIR}/GitHub_repo/Loop_fun_src/build/
mkdir -p ${WORKDIR}/GitHub_repo/Hooks_src/build/
mkdir -p ${WORKDIR}/GitHub_repo/maps/Comparisions/

cd ${WORKDIR}

# Copy script files # REVIEW removed since they'll just be there anyway?
cp ${HOMEDIR}/buzz_scripts/final1.bo ${WORKDIR}/buzz_scripts
cp ${HOMEDIR}/buzz_scripts/final1.bdb ${WORKDIR}/buzz_scripts
cp ${HOMEDIR}/loop_funcs/build/libkh_exp_lf.so ${WORKDIR}/loop_funcs/build/
cp ${HOMEDIR}/GitHub_repo/Loop_fun_src/build/libplanning_exp.so  ${WORKDIR}/GitHub_repo/Loop_fun_src/build/
cp ${HOMEDIR}/GitHub_repo/Hooks_src/build/libconnectivity_controller.so ${WORKDIR}/GitHub_repo/Hooks_src/build/
cp ${HOMEDIR}/GitHub_repo/maps/Comparisions/empty.map ${WORKDIR}/GitHub_repo/maps/Comparisions/

# Set up experiment file
sed -e "s|RANDOMSEED|${RANDOMSEED}|g" \
    -e "s|PATH|${PATH}|g" \
    -e "s|INTER_CAGE_DIST|${INTER_CAGE_DIST}|g" \
    -e "s|MASS_OF_OBJECT|${MASS}|g" \
    -e "s|OUTFILE|${OUTFILE}|g" \
    -e "s|ROBOTS|${ROBOTS}|g" \
    ${HOMEDIR}/template.argos > ${EXPERIMENT}

# Launch ARGoS
time argos3 -c ${EXPERIMENT}

# Copy files back to the data directory
cp -af ${OUTFILE} ${DATADIR}
cp -af ${POSFILE} ${DATADIR}


# Cleanup
# rm -rf ${WORKDIR}
