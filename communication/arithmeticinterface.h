#pragma once

#include <QObject>

class ArithmeticOperations;

class ArithmeticInterface
{
public:
    ArithmeticInterface();

    static ArithmeticInterface *getArithmeticInterfaceHandle();

    bool addInterface(
            const double &parm1, const double &parm2, double &result);
    bool subInterface(
            const double &parm1, const double &parm2, double &result);
    bool mulInterface(
            const double &parm1, const double &parm2, double &result);
    bool divInterface(
            const double &parm1, const double &parm2, double &result);

private:
    static ArithmeticInterface *s_arithmeticInterfaceHandle;

    void callInterfaceLog(
            const QString &funcName, const int &ret,
            QString errorMsg = QString());

    ArithmeticOperations *m_arithmeticOperations;
};
