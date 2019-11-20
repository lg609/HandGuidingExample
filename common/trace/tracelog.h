#pragma once

#include <QObject>
#include <QString>

enum LOG_LEVEL {LL_INFO=0, LL_DEBUG, LL_WARN, LL_ERROR, LL_FATAL};

class TraceLog;

TraceLog& operator <<(TraceLog& traceLogger,const char *str);

#define W_INFO(logEvent) \
    TraceLog::getTraceLogHandle()->writeTrace(LL_INFORMATION, logEvent)

#define W_DEBUG(logEvent) \
    TraceLog::getTraceLogHandle()->writeTrace(LL_DEBUG, logEvent)

#define W_WARN(logEvent) \
    TraceLog::getTraceLogHandle()->writeTrace(LL_WARN, logEvent)

#define W_ERROR(logEvent) \
    TraceLog::getTraceLogHandle()->writeTrace(LL_ERROR, logEvent)

#define W_FATAL(logEvent) \
    TraceLog::getTraceLogHandle()->writeTrace(LL_FATAL, logEvent)

class TraceLog : public QObject
{
    Q_OBJECT

public:
    explicit TraceLog(QObject *parent = 0);

public:
    void writeTrace(LOG_LEVEL level, const QString str);
    void writeTrace(LOG_LEVEL level, const char *str);

    static TraceLog *getTraceLogHandle();

signals:
    void signal_sendTraceInfo(int level, QString info);

private:
    static TraceLog *s_traceLogHandle;
};
