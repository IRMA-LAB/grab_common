
HEADERS += \
    $$PWD/inc/threads.h \
    $$PWD/inc/clocks.h \
    $$PWD/../grabcommon.h

SOURCES += \
    $$PWD/src/threads.cpp \
    $$PWD/src/clocks.cpp \
    $$PWD/tests/libgrabrt_test.cpp \
    $$PWD/../grabcommon.cpp

INCLUDEPATH += \
    $$PWD/inc \
    $$PWD/..

QT       += testlib
QT       -= gui

TARGET = libgrabrt_test

CONFIG   += console c++11
CONFIG   -= app_bundle

TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Lib numeric
unix:!macx: LIBS += -L$$PWD/../libnumeric/lib/ -lnumeric
INCLUDEPATH += $$PWD/../libnumeric $$PWD/../libnumeric/inc/
DEPENDPATH += $$PWD/../libnumeric
unix:!macx: PRE_TARGETDEPS += $$PWD/../libnumeric/lib/libnumeric.a
