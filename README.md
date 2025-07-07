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

