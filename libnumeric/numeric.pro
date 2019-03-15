
HEADERS += \
    $$PWD/inc/matrix.h \
    $$PWD/inc/matrix_utilities.h \
    $$PWD/inc/solvers.h \
    $$PWD/inc/filters.h \
    $$PWD/inc/common.h

SOURCES += \
    $$PWD/src/matrix.cpp \
    $$PWD/src/matrix_utilities.tcc \
    $$PWD/src/solvers.cpp \
    $$PWD/src/filters.cpp

INCLUDEPATH += $$PWD/inc

QT           -= gui

CONFIG   += c++11 staticlib
CONFIG    -= app_bundle

TEMPLATE = lib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
