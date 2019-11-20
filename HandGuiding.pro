include(../../teachpendantplugin.pri)

QT += core gui widgets sql serialport

QMAKE_CXXFLAGS_WARN_ON += -Wno-reorder

include(common/common.pri)
include(sdk/sdk.pri)
include(communication/communication.pri)
include(pluginform/pluginform.pri)
include(utility/utility.pri)
include(ftsensor/ftsensor.pri)

TRANSLATIONS += \
    Resources/Translations/en_GB_template_plugin.ts \
    Resources/Translations/zh_CN_template_plugin.ts \
    Resources/Translations/zh_TW_template_plugin.ts \
    Resources/Translations/fr_FR_template_plugin.ts \
    Resources/Translations/de_DE_template_plugin.ts \

#********linking library **********************************
#unix{
#    #32bit system
#    contains(QT_ARCH, i386){
#        LIBS += -L/home/user/AuboRobotWorkSpace/teachpendant/lib -lauborobotcontroller
##        LIBS += -L$$PWD/lib/lib32/libauborobotcontroller.a
##        LIBS += /home/user/AuboRobotWorkSpace/teachpendant/lib/libotgLib.a
#    }
#    #64bit system
#    contains(QT_ARCH, x86_64){
#        LIBS += -L/home/user/AuboRobotWorkSpace/teachpendant/lib -lauborobotcontroller
##        LIBS += $$PWD/lib/lib64/libotgLib.a
#    }
#}

###********linking library **********************************
unix{
    #32bit system
    contains(QT_ARCH, i386){
        LIBS += -L$$PWD/lib/lib32/ -lauborobotcontroller
#        LIBS += -L$$PWD/lib/lib32/libauborobotcontroller.a
#        LIBS += $$PWD/lib/lib32/libotgLib.a
    }
    #64bit system
    contains(QT_ARCH, x86_64){
        LIBS += -L$$PWD/lib/lib64/ -lauborobotcontroller
#        LIBS += $$PWD/lib/lib64/libotgLib.a
    }
}

INCLUDEPATH += $$PWD/include

HEADERS += \
    HandGuiding.h \
    include/FTSensorDataProcess.h \
    include/robotcontrol.h

SOURCES += \
    HandGuiding.cpp \
    src/FTSensorDataProcess.cpp \
    src/robotcontrol.cpp

INCLUDEPATH += $$processPath($$HEADERS, $$INCLUDEPATH)
DEPENDPATH += $$processPath($$HEADERS, $$DEPENDPATH)
