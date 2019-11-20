!isEmpty(COMMUNICATION_PRI_INCLUDED):error("communication.pri already included")
COMMUNICATION_PRI_INCLUDED = 1

HEADERS += \
    $$PWD/arithmeticinterface.h \
    $$PWD/commondinfo.h \
    $$PWD/communicationthread.h \
    $$PWD/metatypeconversion.h

SOURCES += \
    $$PWD/arithmeticinterface.cpp \
    $$PWD/communicationthread.cpp \
    $$PWD/metatypeconversion.cpp
