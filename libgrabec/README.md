# Libgrabec

## Description

GRAB EtherCAT library includes: 
- A base class for an EtherCAT master node. The master is in charge of setting up the newtork, reading and writing data on it in a synchronous way, respecting a real-time deadline for each cycle, and obviously performing some computation at every cycle depending on the task and the data collected.
- A base class for any EtherCAT slave, providing basic initialization and reading/writing functionalities.
- A set of finalized slaves, such as GoldSoloWhistle drive which provides access to Elmo's corresponding drive.
- A Python [tool](./tools/README.txt) to automatically generate a source/header class of an EasyCAT slave, out of its configuration file. This allows to transform any arduino in a generic EtherCAT slave, once properly configured in few simple steps.

Please note this library is a work in progress, and is not yet meant to be complete, but only essential to the requirements given by the parent project that make use of it.

## Prerequisites

Real-time features required by EtherCAT master can be effectively used if and only if your OS supports real-time scheduling.
Moreover this library relies on _etherlab_ libraries, opportunely patched to work with a linux real-time kernel.
Please refer to [this](https://github.com/UNIBO-GRABLab/cable_robot/wiki/Installation) installation guide of _cable_robot_ wiki to install both the Linux real-time kernel and the external libraries on your machine.

## Compilation

This library depends on _libgrabrt_ static library. Keeping this in mind, the user can freely decide how to import it and integrate it in his/her project.

We provide here two Qt project files for compiling this package as a static library ([grabec.pro](./grabec.pro)) or for unit testing ([libgrabec_test.pro](libgrabec_test.pro)). For the former one, we suggest to build it in a new local folder inside the _libgrabec_ directory, such as "_~/libgrabec/lib/_".

## Usage

If you compiled the library as static as suggested, from the project explorer tab you can right click on your Qt project, select "_Add Library..._" and follow instructions for external libraries. You also need to manually add the include folder of this library (i.e. _~/libgrabec/inc/_) to the `INCLUDEPATH` in your project file (_.pro_), otherwise there will be troubles in file localization when builing the code and including the headers.

To use this library include the following headers according to the functionalities you need:
- `"ethercatmaster.h"` for creating your own master;
- `"ethercatslave.h"` for creating your own slave;
- `"goldsolowhistledrive.h"` for including Elmo's drives in your network.

Please refer to code documentation below to obtain more detailed information about usage of single functions and classes contained in this library.

## Documentation

To auto-generate an user-friendly documentation straight out of the source code, please follow instructions in **Documentation** section [here](../README.md) instructions and substitute _$MYPROJECTPATH_ with _$REPO/cable_robot/lib/grab_common/libgrabec_, where _$REPO_ is the absolute path to your _cable_robot_ repository clone.
