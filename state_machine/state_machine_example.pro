# Created by and for Qt Creator This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

TARGET = Main

HEADERS = \
   $$PWD/examples/CentrifugeTest.h \
   $$PWD/examples/Motor.h \
   $$PWD/examples/MotorNM.h \
   $$PWD/examples/Player.h \
   $$PWD/examples/SelfTest.h \
   $$PWD/inc/allocator.h \
   $$PWD/inc/DataTypes.h \
   $$PWD/inc/Fault.h \
   $$PWD/inc/StateMachine.h \
   $$PWD/inc/stdafx.h \
   $$PWD/inc/targetver.h \
   $$PWD/inc/xallocator.h \
    examples/mybaseclass.h \
    examples/myderivedclass.h

SOURCES = \
   $$PWD/examples/CentrifugeTest.cpp \
   $$PWD/examples/Main.cpp \
   $$PWD/examples/Motor.cpp \
   $$PWD/examples/MotorNM.cpp \
   $$PWD/examples/Player.cpp \
   $$PWD/examples/SelfTest.cpp \
   $$PWD/src/allocator.cpp \
   $$PWD/src/Fault.cpp \
   $$PWD/src/StateMachine.cpp \
   $$PWD/src/stdafx.cpp \
   $$PWD/src/xallocator.cpp \
    examples/mybaseclass.cpp \
    examples/myderivedclass.cpp

INCLUDEPATH = \
    $$PWD/examples \
    $$PWD/inc

#DEFINES = EXTERNAL_EVENT_NO_HEAP_DATA

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

CONFIG += c++11 console
CONFIG -= app_bundle
