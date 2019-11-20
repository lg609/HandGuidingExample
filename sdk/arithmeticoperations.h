#pragma once

#include "metatype.h"

class ArithmeticOperations
{
public:
    ArithmeticOperations() {}

    std::string lastErrorMsg() const;

    int add(ArithmeticParm parm, double &result);
    int sub(ArithmeticParm parm, double &result);
    int mul(ArithmeticParm parm, double &result);
    int div(ArithmeticParm parm, double &result);

private:
    std::string setLastError(const std::string &lastErrorMsg);
    std::string m_lastErrorMsg;
};
