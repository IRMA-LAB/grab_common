
QT       -= gui

CONFIG   += c++11 staticlib
CONFIG   -= app_bundle

TEMPLATE = lib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

HEADERS += \
    ../grabcommon.h \
    inc/ethercatmaster.h \
    inc/ethercatslave.h \
    inc/types.h \
    inc/slaves/easycatslave.h

SOURCES += \
    src/ethercatmaster.cpp \
    src/ethercatslave.cpp \
    src/types.cpp \
    src/slaves/easycatslave.cpp

INCLUDEPATH += inc \
      ../libgrabrt/inc \
      ../ \
      /opt/etherlab/include/

DEPENDPATH  += /opt/etherlab/lib/ ../libgrabrt/build/

LIBS        += /opt/etherlab/lib/libethercat.a ../libgrabrt/build/libgrabrt.a
