TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

GRAB_COMMON_DIR = /home/edoardoida/github/grab_common

HEADERS += \
        ../../../inc/slaves/goldsolowhistledrive.h \
        easycatmaster.h \
        $$GRAB_COMMON_DIR/libgrabec/inc/slaves/easycat/Elmo_minimal_slave_slave.h

SOURCES += \
        ../../../src/slaves/goldsolowhistledrive.cpp \
        main.cpp \
        easycatmaster.cpp \
        $$GRAB_COMMON_DIR/libgrabec/src/slaves/easycat/Elmo_minimal_slave_slave.cpp

INCLUDEPATH += $$GRAB_COMMON_DIR 
TARGET = MinimalEasyCAT

DEFINES += SRCDIR=\"$$PWD/\"

# GRAB Ethercat lib
unix:!macx: LIBS += -L$$GRAB_COMMON_DIR/libgrabec/lib/ -lgrabec
INCLUDEPATH += $$GRAB_COMMON_DIR/libgrabec     $$GRAB_COMMON_DIR/libgrabec/inc
DEPENDPATH += $$GRAB_COMMON_DIR/libgrabec
unix:!macx: PRE_TARGETDEPS += $$GRAB_COMMON_DIR/libgrabec/lib/libgrabec.a

# GRAB Real-time lib
unix:!macx: LIBS += -L$$GRAB_COMMON_DIR/libgrabrt/lib/ -lgrabrt
INCLUDEPATH += $$GRAB_COMMON_DIR/libgrabrt     $$GRAB_COMMON_DIR/libgrabrt/inc
DEPENDPATH += $$GRAB_COMMON_DIR/libgrabrt
unix:!macx: PRE_TARGETDEPS += $$GRAB_COMMON_DIR/libgrabrt/lib/libgrabrt.a

# EtherCAT lib
INCLUDEPATH += /opt/etherlab/include
DEPENDPATH  += /opt/etherlab/lib/
LIBS        += /opt/etherlab/lib/libethercat.a

unix:!macx: LIBS += -L/home/edoardoida/github/state_machine/lib/ -lstate_machine
INCLUDEPATH += /home/edoardoida/github/state_machine /home/edoardoida/github/state_machine/inc
DEPENDPATH += /home/edoardoida/github/state_machine
unix:!macx: PRE_TARGETDEPS += /home/edoardoida/github/state_machine/lib/libstate_machine.a

LIBS += -pthread
