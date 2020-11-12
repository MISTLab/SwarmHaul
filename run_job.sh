#!/bin/bash
#SBATCH --account=def-beltrame
#SBATCH --time=12:00:00
#SBATCH --mem=16G
#SBATCH --cpus-per-task=8

# Stop on any error
set -e

# Basic variables
ME=$(whoami)
# niagra
# BASEWORKDIR=${SCRATCH}/run_dir
# HOMEDIR=${SCRATCH}/collaborative_transport
# DATADIR=${SCRATCH}/data
# Cedar and graham
BASEWORKDIR=${HOME}/scratch/run_dir
HOMEDIR=${HOME}/scratch/collaborative_transport
DATADIR=${HOME}/scratch/data
mkdir -p ${DATADIR}

# Other useful variables
#export ARGOS_PLUGIN_PATH=${HOMEDIR}/build
#export LD_LIBRARY_PATH=${ARGOS_PLUGIN_PATH}:$LD_LIBRARY_PATH

# Script parameters
ROBOTS=${1}
ROBOT_PATH=${2}
INTER_CAGE_DIST=${3}
MASS=${4}
OBJECT_SHAPE=${5}
RANDOMSEED=${6}

# Run id, used as base file name
RUNID=${ROBOTS}_${ROBOT_PATH}_${INTER_CAGE_DIST}_${MASS}_${OBJECT_SHAPE}_${RANDOMSEED}

echo "${RUNID}"

# Output file names and directories
WORKDIR=${BASEWORKDIR}/${ME}_${RUNID}
OUTFILE=${RUNID}.csv
EXPERIMENT=run_${RUNID}.argos

#PATH=/cvmfs/soft.computecanada.ca/easybuild/software/2017/avx512/Compiler/intel2018.3/openmpi/3.1.2/bin:/cvmfs/restricted.computecanada.ca/easybuild/software/2017/Core/ifort/2018.3.222/compilers_and_libraries_2018.3.222/linux/bin/intel64:/cvmfs/restricted.computecanada.ca/easybuild/software/2017/Core/icc/2018.3.222/compilers_and_libraries_2018.3.222/linux/bin/intel64:/cvmfs/soft.computecanada.ca/nix/var/nix/profiles/gcc-7.3.0/bin:/cvmfs/soft.computecanada.ca/easybuild/software/2017/Core/imkl/2018.3.222/mkl/bin:/cvmfs/soft.computecanada.ca/easybuild/software/2017/Core/imkl/2018.3.222/bin:/cvmfs/soft.computecanada.ca/custom/bin/computecanada:/cvmfs/soft.computecanada.ca/easybuild/bin:/cvmfs/soft.computecanada.ca/custom/bin:/cvmfs/soft.computecanada.ca/nix/var/nix/profiles/16.09/bin:/cvmfs/soft.computecanada.ca/nix/var/nix/profiles/16.09/sbin:/opt/slurm/bin:/usr/local/bin:/usr/bin:/usr/local/sbin:/usr/sbin:/scinet/niagara/bin:/usr/lpp/mmfs/bin:/scratch/b/beltrame/viveks/software/bin

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
cp ${HOMEDIR}/GitHub_repo/Loop_fun_src/build/libplanning_exp.so  ${WORKDIR}/GitHub_repo/Loop_fun_src/build/
cp ${HOMEDIR}/GitHub_repo/Hooks_src/build/libconnectivity_controller.so ${WORKDIR}/GitHub_repo/Hooks_src/build/
cp ${HOMEDIR}/GitHub_repo/maps/Comparisions/empty.map ${WORKDIR}/GitHub_repo/maps/Comparisions/

cd ${WORKDIR}

# Set up experiment file
sed -e "s|RANDOMSEED|${RANDOMSEED}|g" \
    -e "s|PATH|${ROBOT_PATH}|g" \
    -e "s|INTERCAGEDIST|${INTER_CAGE_DIST}|g" \
    -e "s|MASS_OF_OBJECT|${MASS}|g" \
    -e "s|OUTFILE|${OUTFILE}|g" \
    -e "s|ROBOTS|${ROBOTS}|g" \
    -e "s|OBJECTTYPE|${OBJECT_SHAPE}|g" \
    ${HOMEDIR}/template.argos > ${EXPERIMENT}

# Launch ARGoS
cd ${WORKDIR}
time argos3 -c ${EXPERIMENT}

# Copy files back to the data directory
cp -af perf_${OUTFILE} ${DATADIR}
cp -af pos_${OUTFILE} ${DATADIR}
cp -af effec_${OUTFILE} ${DATADIR}


# Cleanup
# rm -rf ${WORKDIR}

