#pragma once

#include <iostream>

enum ErrorCode{
    ErrorCode_Succ,

    ErrorCode_DivisorIs0,
};

/** 描述机械臂的路点信息　**/
typedef struct
{
    double parm1;
    double parm2;
}ArithmeticParm;
