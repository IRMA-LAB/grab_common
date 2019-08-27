QT       += testlib
QT       -= gui

TARGET = libcdpr_test
CONFIG   += console c++11
CONFIG   -= app_bundle

TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

HEADERS += \
    $$PWD/inc/kinematics.h \
    $$PWD/inc/diffkinematics.h \
    $$PWD/inc/cdpr_types.h \
    $$PWD/tools/json.hpp \
    $$PWD/tools/robotconfigjsonparser.h \
    $$PWD/inc/statics.h \
    $$PWD/../grabcommon.h


SOURCES += \
    $$PWD/src/kinematics.cpp \
    $$PWD/src/diffkinematics.cpp \
    $$PWD/tools/robotconfigjsonparser.cpp \
    $$PWD/src/statics.cpp \
    $$PWD/tests/libcdpr_test.cpp \
    $$PWD/../grabcommon.cpp

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/tools \
    $$PWD/..

# Lib numeric
unix:!macx: LIBS += -L$$PWD/../libnumeric/lib/ -lnumeric
INCLUDEPATH += $$PWD/../libnumeric $$PWD/../libnumeric/inc
DEPENDPATH += $$PWD/../libnumeric
unix:!macx: PRE_TARGETDEPS += $$PWD/../libnumeric/lib/libnumeric.a

# Lib geometric
unix:!macx: LIBS += -L$$PWD/../libgeom/lib/ -lgeom
INCLUDEPATH += $$PWD/../libgeom $$PWD/../libgeom/inc
DEPENDPATH += $$PWD/../libgeom
unix:!macx: PRE_TARGETDEPS += $$PWD/../libgeom/lib/libgeom.a

# Armadillo lib
LIBS += -llapack -lblas -larmadillo

# Matlab engine
HEADERS += /usr/local/MATLAB/R2019a/extern/include/MatlabEngine.hpp \
           /usr/local/MATLAB/R2019a/extern/include/MatlabDataArray.hpp
unix:!macx: LIBS += -L/usr/local/MATLAB/R2019a/extern/bin/glnxa64/ -lMatlabEngine
unix:!macx: LIBS += -L/usr/local/MATLAB/R2019a/extern/bin/glnxa64/ -lMatlabDataArray
INCLUDEPATH += /usr/local/MATLAB/R2019a/extern/bin/glnxa64 \
           /usr/local/MATLAB/R2019a/extern/include/
DEPENDPATH += /usr/local/MATLAB/R2019a/extern/bin/glnxa64
