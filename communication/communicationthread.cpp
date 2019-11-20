#include "communicationthread.h"
#include "commondinfo.h"
#include <QWaitCondition>
#include <QMutex>
#include <QDebug>
#include "arithmeticinterface.h"

CommunicationThread *CommunicationThread::s_communicationThreadHandle = NULL;

CommunicationThread::CommunicationThread(const bool &isRecvMultiCmd) :
    m_isQuitThread(false), m_isRecvMultiCmd(isRecvMultiCmd)
{
    m_queueMutex = new QMutex;

    m_waitConditionMutex = new QMutex;
    m_waitCondition = new QWaitCondition;

    m_quitConditionMutex = new QMutex;
    m_quitCondition = new QWaitCondition;

    m_commandInfoQueue.clear();

    start();

    s_communicationThreadHandle = this;
}

void CommunicationThread::enqueueCommandInfo(
        QObject *object, const CmdType &cmdType)
{
    CommandBase *commandInfo = new CommandBase;
    commandInfo->m_object = object;
    commandInfo->m_commandType = cmdType;

    enqueueCommandInfo(commandInfo);
}

void CommunicationThread::enqueueCommandInfo_add(
        QObject *object, const double &parm1, const double &parm2)
{
    Command_Add *commandInfo = new Command_Add;
    commandInfo->m_object = object;
    commandInfo->m_commandType = CmdType_Add;
    commandInfo->m_parm1 = parm1;
    commandInfo->m_parm2 = parm2;

    enqueueCommandInfo(commandInfo);
}

void CommunicationThread::enqueueCommandInfo_sub(
        QObject *object, const double &parm1, const double &parm2)
{
    Command_Sub *commandInfo = new Command_Sub;
    commandInfo->m_object = object;
    commandInfo->m_commandType = CmdType_Sub;
    commandInfo->m_parm1 = parm1;
    commandInfo->m_parm2 = parm2;

    enqueueCommandInfo(commandInfo);
}

void CommunicationThread::enqueueCommandInfo_mul(
        QObject *object, const double &parm1, const double &parm2)
{
    Command_Mul *commandInfo = new Command_Mul;
    commandInfo->m_object = object;
    commandInfo->m_commandType = CmdType_Mul;
    commandInfo->m_parm1 = parm1;
    commandInfo->m_parm2 = parm2;

    enqueueCommandInfo(commandInfo);
}

void CommunicationThread::enqueueCommandInfo_div(
        QObject *object, const double &parm1, const double &parm2)
{
    Command_Div *commandInfo = new Command_Div;
    commandInfo->m_object = object;
    commandInfo->m_commandType = CmdType_Div;
    commandInfo->m_parm1 = parm1;
    commandInfo->m_parm2 = parm2;

    enqueueCommandInfo(commandInfo);
}

void CommunicationThread::enqueueCommandInfo(CommandBase *commandInfo)
{
    m_queueMutex->lock();
    if (m_isRecvMultiCmd || m_commandInfoQueue.isEmpty())
        m_commandInfoQueue.enqueue(commandInfo);
    m_queueMutex->unlock();

    m_waitConditionMutex->lock();
    m_waitCondition->wakeOne();
    m_waitConditionMutex->unlock();
}

void CommunicationThread::stopThread()
{
    m_isQuitThread = true;

    bool isTimeOut = false;

    m_quitConditionMutex->lock();
    isTimeOut = m_quitCondition->wait(m_quitConditionMutex, 1000)?
                false : true;
    m_quitConditionMutex->unlock();

    if (isTimeOut) {
        qDebug() << "Timed out waiting for thread to exit";
        this->terminate();
        this->wait(1000);
    }

    clearCommandInfoQueue();
}

void CommunicationThread::run()
{
    while (!m_isQuitThread)
    {
        m_waitConditionMutex->lock();
        m_waitCondition->wait(m_waitConditionMutex, 50);
        m_waitConditionMutex->unlock();

        while (!m_commandInfoQueue.isEmpty())
        {
            CommandBase *commandInfo = m_commandInfoQueue.first();
            processTasks(commandInfo);

            if (commandInfo)
            {
                m_queueMutex->lock();
                m_commandInfoQueue.dequeue();
                m_queueMutex->unlock();

                delete commandInfo;
                commandInfo = NULL;
            }
        }
    }

    m_quitConditionMutex->lock();
    m_quitCondition->wakeOne();
    m_quitConditionMutex->unlock();

    qDebug() << "Quit thread: " << QThread::currentThread();
}

void CommunicationThread::clearCommandInfoQueue()
{
    while (!m_commandInfoQueue.isEmpty())
        delete m_commandInfoQueue.dequeue();
}

void CommunicationThread::processTasks(CommandBase *commandInfo)
{
    switch (commandInfo->m_commandType) {
    case CmdType_Add:
    {
        Command_Add *cmd = dynamic_cast<Command_Add *>(commandInfo);
        double result = 0.0;
        bool isSuccess
                = ArithmeticInterface::getArithmeticInterfaceHandle()
                ->addInterface(cmd->m_parm1, cmd->m_parm2, result);
        emit signal_sendCall_addInterface_result(
                    commandInfo->m_object, isSuccess, result);

        break;
    }
    case CmdType_Sub:
    {
        Command_Sub *cmd = dynamic_cast<Command_Sub *>(commandInfo);
        double result = 0.0;
        bool isSuccess
                = ArithmeticInterface::getArithmeticInterfaceHandle()
                ->subInterface(cmd->m_parm1, cmd->m_parm2, result);
        emit signal_sendCall_subInterface_result(
                    commandInfo->m_object, isSuccess, result);

        break;
    }
    case CmdType_Mul:
    {
        Command_Mul *cmd = dynamic_cast<Command_Mul *>(commandInfo);
        double result = 0.0;
        bool isSuccess
                = ArithmeticInterface::getArithmeticInterfaceHandle()
                ->mulInterface(cmd->m_parm1, cmd->m_parm2, result);
        emit signal_sendCall_mulInterface_result(
                    commandInfo->m_object, isSuccess, result);

        break;
    }
    case CmdType_Div:
    {
        Command_Div *cmd = dynamic_cast<Command_Div *>(commandInfo);
        double result = 0.0;
        bool isSuccess
                = ArithmeticInterface::getArithmeticInterfaceHandle()
                ->divInterface(cmd->m_parm1, cmd->m_parm2, result);
        emit signal_sendCall_divInterface_result(
                    commandInfo->m_object, isSuccess, result);

        break;
    }
    default:
        break;
    }
}
