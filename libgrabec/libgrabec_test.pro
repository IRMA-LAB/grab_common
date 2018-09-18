# Unit tests
SOURCES += \
        tests/libgrabec_test.cpp

# Qt unit-test config
QT       += testlib
QT       -= gui

TARGET = libgrabec_test

CONFIG   += c++11 console
CONFIG   -= app_bundle

TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Include grabec library
unix:!macx: LIBS += -L$$PWD/build/ -lgrabec

INCLUDEPATH += $$PWD/build $$PWD/../ $$PWD/inc $$PWD/../libgrabrt/inc
DEPENDPATH += $$PWD/build

unix:!macx: PRE_TARGETDEPS += $$PWD/build/libgrabec.a

