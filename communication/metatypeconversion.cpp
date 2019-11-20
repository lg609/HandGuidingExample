#include "metatypeconversion.h"

ArithmeticParm MetaTypeConversion::pluginToSdk_arithmeticParm(
        const double &parm1, const double &parm2)
{
    ArithmeticParm dest;

    dest.parm1 = parm1;
    dest.parm2 = parm2;

    return dest;
}
