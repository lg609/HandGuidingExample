#include "tracelog.h"
#include <stdio.h>
#include <iostream>
#include <QDebug>

TraceLog *TraceLog::s_traceLogHandle = NULL;

TraceLog *TraceLog::getTraceLogHandle()
{
    if (NULL == s_traceLogHandle)
        s_traceLogHandle = new TraceLog;

    return s_traceLogHandle;
}

TraceLog::TraceLog(QObject *parent) :
    QObject(parent)
{
}

void TraceLog::writeTrace(LOG_LEVEL level, const QString str)
{
    emit signal_sendTraceInfo(level, str);
}

void TraceLog::writeTrace(LOG_LEVEL level, const char *str)
{
    emit signal_sendTraceInfo(level, str);
}

TraceLog &operator <<(TraceLog &traceLogger, const char *str)
{
    traceLogger.writeTrace(LL_INFO, str);

    return traceLogger;
}
