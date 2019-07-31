
HEADERS += \
    $$PWD/inc/kinematics.h \
    $$PWD/inc/diffkinematics.h \
    $$PWD/inc/cdpr_types.h \
    $$PWD/tools/json.hpp \
    $$PWD/tools/robotconfigjsonparser.h \
    $$PWD/inc/statics.h

SOURCES += \
    $$PWD/src/kinematics.cpp \
    $$PWD/src/diffkinematics.cpp \
    $$PWD/tools/robotconfigjsonparser.cpp \
    $$PWD/src/statics.cpp

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/tools

QT       -= gui

CONFIG   += staticlib c++11
CONFIG   -= app_bundle

TEMPLATE = lib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

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

# OpenCV lib
INCLUDEPATH += /usr/local/lib/opencv-4.0.1/build/include \
               /usr/local/include/opencv4
LIBS += -L"/usr/local/lib/opencv-4.0.1/build/lib"
LIBS += -lopencv_core
