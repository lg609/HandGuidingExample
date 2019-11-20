QT+=widgets

INCLUDEPATH += $$PWD\include

HEADERS += \
    $$PWD/include/ftsensor.h \
    $$PWD/include/OptoForceSensor.h \
    $$PWD/include/RobotiqSensor.h \
    $$PWD/include/ATISensor.h \
    $$PWD/include/KunWeiSensor.h \
    $$PWD/include/serial.h \

SOURCES += \
    $$PWD/src/ftsensor.cpp \
    $$PWD/src/OptoForceSensor.cpp \
    $$PWD/src/RobotiqSensor.cpp \
    $$PWD/src/ATISensor.cpp \
    $$PWD/src/KunWeiSensor.cpp \
    $$PWD/src/serial.cpp \

