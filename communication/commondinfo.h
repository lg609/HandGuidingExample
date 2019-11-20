#pragma once

#include <QString>
#include <QObject>

enum CmdType
{
    CmdType_None,

    CmdType_Add,
    CmdType_Sub,
    CmdType_Mul,
    CmdType_Div,
};

class CommandBase
{
public:
    CommandBase() {}
    virtual ~CommandBase() {}

    QObject *m_object;
    CmdType m_commandType;
};

class Command_Add : public CommandBase
{
public:
    Command_Add() {}
    virtual ~Command_Add() {}

    double m_parm1;
    double m_parm2;
};

class Command_Sub : public CommandBase
{
public:
    Command_Sub() {}
    virtual ~Command_Sub() {}

    double m_parm1;
    double m_parm2;
};

class Command_Mul : public CommandBase
{
public:
    Command_Mul() {}
    virtual ~Command_Mul() {}

    double m_parm1;
    double m_parm2;
};

class Command_Div : public CommandBase
{
public:
    Command_Div() {}
    virtual ~Command_Div() {}

    double m_parm1;
    double m_parm2;
};
