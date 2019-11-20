#pragma once

#include <QThread>
#include <QQueue>
#include "commondinfo.h"

class ControlInterface;
class QMutex;
class QWaitCondition;
class CommandBase;

class CommunicationThread : public QThread
{
    Q_OBJECT

public:
    explicit CommunicationThread(const bool &isRecvMultiCmd);

    static CommunicationThread *getCommunicationThreadHandle()
    {return s_communicationThreadHandle;}

    void enqueueCommandInfo(QObject *object, const CmdType &cmdType);
    void enqueueCommandInfo_add(
            QObject *object, const double &parm1, const double &parm2);
    void enqueueCommandInfo_sub(
            QObject *object, const double &parm1, const double &parm2);
    void enqueueCommandInfo_mul(
            QObject *object, const double &parm1, const double &parm2);
    void enqueueCommandInfo_div(
            QObject *object, const double &parm1, const double &parm2);

    void stopThread();

protected:
    void run();

signals:
    void signal_sendCall_addInterface_result(
            QObject *object, bool isSuccess, double result);
    void signal_sendCall_subInterface_result(
            QObject *object, bool isSuccess, double result);
    void signal_sendCall_mulInterface_result(
            QObject *object, bool isSuccess, double result);
    void signal_sendCall_divInterface_result(
            QObject *object, bool isSuccess, double result);

private:
    static CommunicationThread *s_communicationThreadHandle;

    QMutex *m_waitConditionMutex;
    QWaitCondition *m_waitCondition;

    QMutex *m_queueMutex;
    QQueue <CommandBase *> m_commandInfoQueue;

    QMutex * m_quitConditionMutex;
    QWaitCondition *m_quitCondition;

    bool m_isRecvMultiCmd;

    void clearCommandInfoQueue();

    bool m_isQuitThread;

    void processTasks(CommandBase *commandInfo);

    void enqueueCommandInfo(CommandBase *commandInfo);
};
