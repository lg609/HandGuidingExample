#include "arithmeticinterface.h"
#include <QDir>
#include "tracelog.h"
#include <QDebug>
#include "arithmeticoperations.h"
#include "metatypeconversion.h"

ArithmeticInterface *ArithmeticInterface::s_arithmeticInterfaceHandle = NULL;

ArithmeticInterface *ArithmeticInterface::getArithmeticInterfaceHandle()
{
    if (NULL == s_arithmeticInterfaceHandle)
        s_arithmeticInterfaceHandle = new ArithmeticInterface;

    return s_arithmeticInterfaceHandle;
}

ArithmeticInterface::ArithmeticInterface()
{
    m_arithmeticOperations = new ArithmeticOperations;
}

bool ArithmeticInterface::addInterface(
        const double &parm1, const double &parm2, double &result)
{
    int ret = m_arithmeticOperations->add(
                MetaTypeConversion::pluginToSdk_arithmeticParm(parm1, parm2),
                result);
    if (ErrorCode_Succ != ret)
        callInterfaceLog(
                    __FUNCTION__, ret,
                    QString::fromStdString(
                        m_arithmeticOperations->lastErrorMsg()));
    return ErrorCode_Succ == ret;
}

bool ArithmeticInterface::subInterface(
        const double &parm1, const double &parm2, double &result)
{
    int ret = m_arithmeticOperations->sub(
                MetaTypeConversion::pluginToSdk_arithmeticParm(parm1, parm2),
                result);
    if (ErrorCode_Succ != ret)
        callInterfaceLog(
                    __FUNCTION__, ret,
                    QString::fromStdString(
                        m_arithmeticOperations->lastErrorMsg()));
    return ErrorCode_Succ == ret;
}

bool ArithmeticInterface::mulInterface(
        const double &parm1, const double &parm2, double &result)
{
    int ret = m_arithmeticOperations->mul(
                MetaTypeConversion::pluginToSdk_arithmeticParm(parm1, parm2),
                result);
    if (ErrorCode_Succ != ret)
        callInterfaceLog(
                    __FUNCTION__, ret,
                    QString::fromStdString(
                        m_arithmeticOperations->lastErrorMsg()));
    return ErrorCode_Succ == ret;
}

bool ArithmeticInterface::divInterface(
        const double &parm1, const double &parm2, double &result)
{
    int ret = m_arithmeticOperations->div(
                MetaTypeConversion::pluginToSdk_arithmeticParm(parm1, parm2),
                result);
    if (ErrorCode_Succ != ret)
        callInterfaceLog(
                    __FUNCTION__, ret,
                    QString::fromStdString(
                        m_arithmeticOperations->lastErrorMsg()));
    return ErrorCode_Succ == ret;
}

void ArithmeticInterface::callInterfaceLog(
        const QString &funcName, const int &ret, QString errorMsg)
{
    W_WARN(QString("Call template plugin interface failed!\
                   Function name : %1,\
                   return value : %2, \
                   error info : %3")
                   .arg(funcName)
                   .arg(ret)
                   .arg(errorMsg));
}
