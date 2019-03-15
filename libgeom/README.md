# Libgeom

## Description

The GRAB geometric library includes:
- Rotations utilities, in different angle parametrizations;
- Minimal quaternion class implementation, with conversion to and from euler angles.

Please note this library is a work in progress, and is not yet meant to be complete, but only essential to the requirements given by the parent project that make use of it.

## Compilation

This library depends on [libnumeric](../libnumeric). Keeping this in mind, the user is free to integrate it in his/her own project as desired.

We provide here two Qt project files for compiling this package as a static library ([geom.pro](./geom.pro)) or for unit testing ([libgeom_test.pro](libgeom_test.pro)). We suggest to build it in a new local folder inside the _libgeom_ directory for the former case, such as "_~/libgeom/lib/_". Please note that this builing process requires a compiled version of static library [libnumeric](../libnumeric).

## Usage

If you compiled the library as static as suggested, from the project explorer tab you can right click on your Qt project, select "_Add Library..._" and follow instructions for external libraries. You also need to manually add the include folder of this library (i.e. _~/libgeom/inc/_) to the `INCLUDEPATH` in your project file (_.pro_), otherwise there will be troubles in file localization when builing the code and including the headers.

To use this library include the following headers according to the functionalities you need:
- `"rotations.h"` for rotations utilities;
- `"quaternions.h"` for quaternions.

Please refer to code documentation below to obtain more detailed information about usage of single functions and classes contained in this library.

## Documentation

To auto-generate an user-friendly documentation straight out of the source code, please follow instructions in **Documentation** section [here](../README) instructions and substitute _$MYPROJECTPATH_ with _$REPO/cable_robot/lib/grab_common/libgeom_, where _$REPO_ is the absolute path to your _cable_robot_ repository clone.
