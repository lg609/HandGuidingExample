#include "arithmeticoperations.h"
#include "unistd.h"

std::string ArithmeticOperations::lastErrorMsg() const
{
    return m_lastErrorMsg;
}

int ArithmeticOperations::add(
        ArithmeticParm parm, double &result)
{
    usleep(1000 * 1000 * 1);
    result = parm.parm1 + parm.parm2;
    return ErrorCode_Succ;
}

int ArithmeticOperations::sub(
        ArithmeticParm parm, double &result)
{
    usleep(1000 * 1000 * 1);
    result = parm.parm1 - parm.parm2;
    return ErrorCode_Succ;
}

int ArithmeticOperations::mul(
        ArithmeticParm parm, double &result)
{
    usleep(1000 * 1000 * 1);
    result = parm.parm1 * parm.parm2;
    return ErrorCode_Succ;
}

int ArithmeticOperations::div(
        ArithmeticParm parm, double &result)
{
    int ret = ErrorCode_Succ;

    if (0 == parm.parm2)
    {
        setLastError("Divisor is 0");
        ret = ErrorCode_DivisorIs0;
    }
    else
    {
        usleep(1000 * 1000 * 1);
        result = parm.parm1 / parm.parm2;
    }

    return ret;
}

std::string ArithmeticOperations::setLastError(
        const std::string &lastErrorMsg)
{
    m_lastErrorMsg = lastErrorMsg;

    if (m_lastErrorMsg.size() > 0)
        std::cerr<<m_lastErrorMsg<<std::endl;

    return m_lastErrorMsg;
}
