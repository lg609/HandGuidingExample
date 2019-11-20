#pragma once

#include <QObject>

enum KeyboardType{
    KeyboardType_NoType,
    KeyboardType_DigitalKeyboard,
    KeyboardType_LogicalExpressionKeyboard,
    KeyboardType_FullKeyboard
};

class Common{
public:
    Common();

    static Common *getCommonHandle();

    QString m_pluginPath;

private:
    static Common *s_commonHandle;
};
