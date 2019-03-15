# Libnumeric

## Description

GRAB numeric library includes: 
- A minimal matrix class implementation with basic operations, element access, block manipulation and utilities. A particular sub-space of this class is represented by single row or column matrices, which are intuitively identified with the alias `Vector`. 
The matrix class is templated, therefore both dimensions and type need to be specified statically at construction time. Some standard type, such as `int` and `double`, are readily available using the respective alias. Aliases for classic square 2x2 and 3x3 matrices are present too.
- Signal filters, in particular at the moment only a simple _low-pass filter_ is implemented.
- Numerical solvers.

Please note this library is a work in progress, and is not yet meant to be complete, but only essential to the requirements given by the parent project that make use of it.

## Compilation

This library is stand-alone and the user can freely decide how to import it and integrate it in his/her project.
We provide here two Qt project files for compiling this package as a static library ([numeric.pro](./numeric.pro)) or for unit testing ([libnumeric_test.pro](libnumeric_test.pro)). For the former one, we suggest to build it in a new local folder inside the _libnumeric_ directory, such as "_~/libnumeric/lib/_".

## Usage

If you compiled the library as static as suggested, from the project explorer tab you can right click on your Qt project, select "_Add Library..._" and follow instructions for external libraries. You also need to manually add the include folder of this library (i.e. _~/libnumeric/inc/_) to the `INCLUDEPATH` in your project file (_.pro_), otherwise there will be troubles in file localization when builing the code and including the headers.

To use this library include the following headers according to the functionalities you need:
- `"matrix_utilities.h"` for the matrix class and utilities;
- `"filters.h"` for the signal filters;
- `"solvers.h"` for the numerical solvers.

Please refer to code documentation below to obtain more detailed information about usage of single functions and classes contained in this library.

## Documentation

To auto-generate an user-friendly documentation straight out of the source code, please follow instructions in **Documentation** section [here](../README.md) instructions and substitute _$MYPROJECTPATH_ with _$REPO/cable_robot/lib/grab_common/libnumeric_, where _$REPO_ is the absolute path to your _cable_robot_ repository clone.
