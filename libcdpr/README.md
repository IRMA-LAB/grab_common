# Libcdpr

## Description

The GRAB CDPR library includes:
- Differential kinematics of order 0, 1 and 2 of a generic cable-driven parallel robot.
- Robot components and parameters structures and types.

Please note this library is a work in progress, and is not yet meant to be complete, but only essential to the requirements given by the parent project that make use of it.

## Compilation

This library depends on [libgeom](../libgeom). Keeping this in mind, the user is free to integrate it in his/her own project as desired.

We provide here two Qt project files for compiling this package as a static library ([cdpr.pro](./cdpr.pro)) or for unit testing ([libcdpr_test.pro](libcdpr_test.pro)). For the former case, we suggest to build it in a new local folder inside the _libcdpr_ directory , such as "_~/libcdpr/lib/_". Please note that this builing process requires a compiled version of static library [libgeom](../libgeom).

## Usage

If you compiled the library as static as suggested, from the project explorer tab you can right click on your Qt project, select "_Add Library..._" and follow instructions for external libraries. You also need to manually add the include folder of this library (i.e. _~/libcdpr/inc/_) to the `INCLUDEPATH` in your project file (_.pro_), otherwise there will be troubles in file localization when builing the code and including the headers.

To use this library include the following headers according to the functionalities you need:
- `"kinematics.h"` for zero-order kinematics of a generic CDPR;
- `"diffkinematics.h"` for first and second-order kinematics of a generic CDPR;
- `"types.h"` for robot components and parameters structures.

Please refer to code documentation below to obtain more detailed information about usage of single functions and classes contained in this library.

## Documentation

To auto-generate an user-friendly documentation straight out of the source code, please follow instructions in **Documentation** section [here](../README) instructions and substitute _$MYPROJECTPATH_ with _$REPO/cable_robot/lib/grab_common/libcdpr_, where _$REPO_ is the absolute path to your _cable_robot_ repository clone.
