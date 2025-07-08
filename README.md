To use this repo first clone the repo and build using the following commands: 

## Install the Argos3 simulator and Buzz
Follow the instructions in this page to install/build the programming language Buzz and Argos3 [github.com/buzz-lang/Buzz/blob/master/doc/argos-integration.md](https://github.com/buzz-lang/Buzz/blob/master/doc/argos-integration.md)

## Install the Khepera Plugin 
Install the khepera robot plugin for Argos3. These are the robots that will perform the collective transport
https://github.com/ilpincy/argos3-kheperaiv

## Install the Collective Transport

```
git clone http://git.mistlab.ca/vvaradharajan/collaborative_transport.git

cd collaborative_transport/GitHub_repo/Hooks_src/
mkdir build
cd build
cmake ..
make 

cd collaborative_transport/GitHub_repo/Loop_fun_src/
mkdir build
cd build
cmake ..
make 
```
## Running the experiments


1. Compile the buzz script:

```
cd buzz_scripts
bzzc -I includes/ Simulation.bzz
cd ..
```
2. run the associated argos file 
```
argos3 -c experiment.argos

# How to control the arguments in the experiment
<loop_functions library="GitHub_repo/Loop_fun_src/build/libplanning_exp.so" 
 label="Planning"
 map_file_name="GitHub_repo/maps/Comparisions/empty.map" 
 map_option="0"
 robots="ROBOTS" # No of robots check below
 out_file="OUTFILE" # suffix to add to out files containg data for plotting
 path="PATH" # can be straight, zigzac or straight_rot
 inter_cage_dist="INTERCAGEDIST" # distance between caging
 random_seed_set="RANDOMSEED" # random seed for the experiment
 object_type="OBJECTTYPE"/> # type of object 

Object type enum 
0 - square object of size (2,2) for 25 robots
1 - square object of size (3.6,6) for 50 robots 
2 - square object of size (7.2,18) for 100 robots
3 - cloud shape for caging tests
4 - box_rotation shape for caging tests
5 - clover shape for caging tests - needs 50 robots
```

## SIMULATION GIF's
<p align="center">
  <img src="https://git.mistlab.ca/vvaradharajan/collaborative_transport/-/raw/master/videos/25.gif?ref_type=heads" width="49%"/>
  <img src="https://git.mistlab.ca/vvaradharajan/collaborative_transport/-/raw/master/videos/50.gif?ref_type=heads" width="49%"/>
</p>

## Hardware GIF's 

<p align="center">
  <img src="https://git.mistlab.ca/vvaradharajan/collaborative_transport/-/raw/master/videos/hardware.gif?ref_type=heads" width="49%"/>
</p>

## Cite us

If you use this work, kindly consider citing us.

```
@InProceedings{10.1007/978-3-030-92790-5_27,
author="Vardharajan, Vivek Shankar
and Soma, Karthik
and Beltrame, Giovanni",
editor="Matsuno, Fumitoshi
and Azuma, Shun-ichi
and Yamamoto, Masahito",
title="Collective Transport via Sequential Caging",
booktitle="Distributed Autonomous Robotic Systems",
year="2022",
publisher="Springer International Publishing",
address="Cham",
pages="349--362",
isbn="978-3-030-92790-5"
}
```