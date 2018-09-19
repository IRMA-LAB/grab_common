
QT       -= gui

CONFIG   += c++11 staticlib
CONFIG   -= app_bundle

TEMPLATE = lib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

HEADERS = \
   $$PWD/inc/allocator.h \
   $$PWD/inc/DataTypes.h \
   $$PWD/inc/Fault.h \
   $$PWD/inc/StateMachine.h \
   $$PWD/inc/stdafx.h \
   $$PWD/inc/targetver.h \
   $$PWD/inc/xallocator.h

SOURCES = \
   $$PWD/src/allocator.cpp \
   $$PWD/src/Fault.cpp \
   $$PWD/src/StateMachine.cpp \
   $$PWD/src/stdafx.cpp \
   $$PWD/src/xallocator.cpp

INCLUDEPATH = $$PWD/inc
