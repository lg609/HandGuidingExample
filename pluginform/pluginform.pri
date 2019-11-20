!isEmpty(PLUGINFORM_PRI_INCLUDED):error("pluginform.pri already included")
PLUGINFORM_PRI_INCLUDED = 1

FORMS += \
    $$PWD/pluginform.ui \
    $$PWD/HandGuidingForm.ui

HEADERS += \
    $$PWD/pluginform.h \
    $$PWD/HandGuidingForm.h

SOURCES += \
    $$PWD/pluginform.cpp \
    $$PWD/HandGuidingForm.cpp
