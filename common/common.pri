!isEmpty(COMMON_PRI_INCLUDED):error("common.pri already included")
COMMON_PRI_INCLUDED = 1

HEADERS += \
    $$PWD/trace/tracelog.h \
    $$PWD/common.h \
    $$PWD/customcomponent/messagebox.h \

SOURCES += \
    $$PWD/trace/tracelog.cpp \
    $$PWD/common.cpp \
    $$PWD/customcomponent/messagebox.cpp \
