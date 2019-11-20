!isEmpty(SDK_PRI_INCLUDED):error("sdk.pri already included")
SDK_PRI_INCLUDED = 1

HEADERS += \
    $$PWD/arithmeticoperations.h \
    $$PWD/metatype.h

SOURCES += \
    $$PWD/arithmeticoperations.cpp
