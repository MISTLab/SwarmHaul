To use this repo first clone the repo and build using the following commands: 

## Install the Argos3 simulator and Buzz
Follow the instructions in this page to install the programming language Buzz and Argos3 [github.com/buzz-lang/Buzz/blob/master/doc/argos-integration.md](https://github.com/buzz-lang/Buzz/blob/master/doc/argos-integration.md)

## Install the Collective Transport

```
git clone http://git.mistlab.ca/vvaradharajan/collaborative_transport.git

cd collaborative_transport
mkdir build
cd build
cmake ../loop_fun_src/
make 
```
The above commands build the loop function in the repository. This loop function is used to obtain the object position of the box entity named "push_object" and update its position inside the buzz virtual machine.
The position of the box entity can be accessed in a buzz script using the keyword "Pushed_obj_pos". "Pushed_obj_pos" is a table with "x" and "y" entry corresponding to the position of the box entity in the global coordinate system.
Pushed_obj_pos.x in a buzz script provides the x coordinate of the object.

To run the example script use the following command: 

Navigate to the repo root and use the following command

1. Compile the example buzz script:

```
cd buzz_scripts

bzzc Object_movement_test.bzz

cd ..

```
2. run the associated argos file 
```
argos3 -c testbuzzexperiment.argos
```

This demo should print the position of the pushed object in the argos log terminal and have all the robots move to a target location (1,0)