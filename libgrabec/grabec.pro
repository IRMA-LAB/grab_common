QT       -= gui

CONFIG   += c++11 staticlib
CONFIG   -= app_bundle

TEMPLATE = lib

DEFINES += USE_QT=1

HEADERS += \
    $$PWD/../grabcommon.h \
    $$PWD/inc/ethercatmaster.h \
    $$PWD/inc/ethercatslave.h \
    $$PWD/inc/grabec_types.h \
    $$PWD/inc/slaves/goldsolowhistledrive.h \
    $$PWD/inc/slaves/mdxservoecslave.h \
    $$PWD/inc/slaves/xsensahrsec.h \
    $$PWD/inc/slaves/xelbssctirma8.h \

SOURCES += \
    $$PWD/../grabcommon.cpp \
    $$PWD/src/ethercatmaster.cpp \
    $$PWD/src/ethercatslave.cpp \
    $$PWD/src/slaves/goldsolowhistledrive.cpp \
    $$PWD/src/slaves/mdxservoecslave.cpp \
    $$PWD/src/slaves/xsensahrsec.cpp \
    $$PWD/src/slaves/xelbssctirma8.cpp \


INCLUDEPATH += \
      $$PWD/inc \
      ../

# Ethercat lib
LIBS        += /opt/etherlab/lib/libethercat.a
INCLUDEPATH += /opt/etherlab/include/
DEPENDPATH  += /opt/etherlab/lib/

# Lib real-time
unix:!macx: LIBS += -L$$PWD/../libgrabrt/lib/ -lgrabrt

INCLUDEPATH += $$PWD/../libgrabrt $$PWD/../libgrabrt/inc
DEPENDPATH += $$PWD/../libgrabrt

unix:!macx: PRE_TARGETDEPS += $$PWD/../libgrabrt/lib/libgrabrt.a

# Lib state-machine
unix:!macx: LIBS += -L$$PWD/../../state_machine/lib/ -lstate_machine

INCLUDEPATH += $$PWD/../../state_machine $$PWD/../../state_machine/inc
DEPENDPATH += $$PWD/../../state_machine

unix:!macx: PRE_TARGETDEPS += $$PWD/../../state_machine/lib/libstate_machine.a
