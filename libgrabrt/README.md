# Libgrabrt

## Description

GRAB real-time library includes: 
- Clocks implementations, with a standard clock to measure elapsed time, and one meant for cyclic functions;
- _pthread_-based class for creating a new thread with optional real-time features and cyclic run function.

Please note this library is a work in progress, and is not yet meant to be complete, but only essential to the requirements given by the parent project that make use of it.

## Prerequisites

Real-time features can be effectively used if and only if your OS supports real-time scheduling.
Please refer to [this](https://github.com/UNIBO-GRABLab/cable_robot#install-real-time-kernel) section of _cable_robot_ repository to install Linux real-time kernel on your machine.

## Compilation

This library is stand-alone and the user can freely decide how to import it and integrate it in his/her project.
We provide here two Qt project files for compiling this package as a static library ([grabrt.pro](./grabrt.pro)) or for unit testing ([libgrabrt_test.pro](libgrabrt_test.pro)). For the former one, we suggest to build it in a new local folder inside the _libgrabrt_ directory, such as "_~/libgrabrt/lib/_".

## Usage

If you compiled the library as static as suggested, from the project explorer tab you can right click on your Qt project, select "_Add Library..._" and follow instructions for external libraries. You also need to manually add the include folder of this library (i.e. _~/libgrabrt/inc/_) to the `INCLUDEPATH` in your project file (_.pro_), otherwise there will be troubles in file localization when builing the code and including the headers.

To use this library include the following headers according to the functionalities you need:
- `"clocks.h"` for clock utilities;
- `"threads.h"` for thread utilities.

Please refer to code documentation below to obtain more detailed information about usage of single functions and classes contained in this library.

## Documentation

To auto-generate an user-friendly documentation straight out of the source code, please follow instructions in **Documentation** section [here](../README) instructions and substitute _$MYPROJECTPATH_ with _$REPO/cable_robot/lib/grab_common/libgrabrt_, where _$REPO_ is the absolute path to your _cable_robot_ repository clone.
