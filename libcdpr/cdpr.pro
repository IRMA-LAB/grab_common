
HEADERS += \
    $$PWD/inc/kinematics.h \
    $$PWD/inc/diffkinematics.h \
    $$PWD/inc/types.h \
    $$PWD/inc/json.hpp

SOURCES += \
    $$PWD/src/kinematics.cpp \
    $$PWD/src/diffkinematics.cpp

INCLUDEPATH += inc

LIBS += ../libgeom/build/libgeom.a ../libnumeric/build/libnumeric.a

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
unix:!macx: LIBS += -L$$PWD/../libnumeric/build/ -lnumeric

INCLUDEPATH += $$PWD/../libnumeric/build $$PWD/../libnumeric/inc
DEPENDPATH += $$PWD/../libnumeric/build

unix:!macx: PRE_TARGETDEPS += $$PWD/../libnumeric/build/libnumeric.a

# Lib geometric
unix:!macx: LIBS += -L$$PWD/../libgeom/build/ -lgeom

INCLUDEPATH += $$PWD/../libgeom/build $$PWD/../libgeom/inc
DEPENDPATH += $$PWD/../libgeom/build

unix:!macx: PRE_TARGETDEPS += $$PWD/../libgeom/build/libgeom.a
