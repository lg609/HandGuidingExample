#include "HandGuiding.h"
#include <QDebug>
#include "tracelog.h"
#include <QDir>
#include "common.h"
#include "messagebox.h"
#include "communicationthread.h"
#include "arithmeticinterface.h"
#include <QTranslator>
#include <QTimer>
#include <QApplication>
#include "pluginform.h"

HandGuiding::HandGuiding(QObject *)
{
    m_translator = new QTranslator(qApp);

    QDir::setCurrent(QCoreApplication::applicationDirPath());

    //RegisterMetaType  qRegisterMetaType <XXX> ("XXX");

    QObject::tr("Peripheral");
    QObject::tr("Template");

    new CommunicationThread(false);
}

bool HandGuiding::pluginCommonInterface(
        const QString &data, QString &retValueOrError)
{
    bool ret = false;

    QString parm = data.simplified().remove(" ");
    QString funcName = parm.section("|", 0, 0);
    QString funcParms = parm.section("|", 1, 1);

    if ("add" == funcName)
    {
        double ioValue;
        if ((ret = add(funcParms.section(",", 0, 0).toDouble(),
                       funcParms.section(",", 1, 1).toDouble(), ioValue)))
            retValueOrError = QString("d|%1").arg(ioValue);
        else
            retValueOrError = "Call add failed";
    }
    else if ("sub" == funcName)
    {
        double ioValue;
        if ((ret = sub(funcParms.section(",", 0, 0).toDouble(),
                       funcParms.section(",", 1, 1).toDouble(), ioValue)))
            retValueOrError = QString("d|%1").arg(ioValue);
        else
            retValueOrError = "Call sub failed";
    }
    else if ("mul" == funcName)
    {
        double ioValue;
        if ((ret = mul(funcParms.section(",", 0, 0).toDouble(),
                       funcParms.section(",", 1, 1).toDouble(), ioValue)))
            retValueOrError = QString("d|%1").arg(ioValue);
        else
            retValueOrError = "Call mul failed";
    }
    else if ("div" == funcName)
    {
        double ioValue;
        if ((ret = div(funcParms.section(",", 0, 0).toDouble(),
                       funcParms.section(",", 1, 1).toDouble(), ioValue)))
            retValueOrError = QString("d|%1").arg(ioValue);
        else
            retValueOrError = "Call div failed";
    }
    else if ("takeFirstWaypoint" == funcName)
    {
        QString firstWaypoint;
        ret = takeFirstWaypoint(firstWaypoint);
        retValueOrError = QString("(dddddd)|%1").arg(firstWaypoint);
    }
    else if ("takePhoto" == funcName)
    {
        ret = takePhoto();
    }

    return ret;
}

QWidget *HandGuiding::createUI(QWidget *parent)
{
    connect(TraceLog::getTraceLogHandle(),
            SIGNAL(signal_sendTraceInfo(int,QString)),
            parent, SIGNAL(signal_sendTraceInfo(int,QString)));

    return new PluginForm(parent);
}

void HandGuiding::setPluginMessageBoxParentWidget(
        QWidget *messageBoxParent)
{
    MessageBox::s_messageBoxParent = messageBoxParent;
}

void HandGuiding::setPluginSharedFilePath(const QString &pluginPath)
{
    //所有的资源文件放到宿主注定的插件路径中. 比如数据库
    Common::getCommonHandle()->m_pluginPath = pluginPath;

    //Create RecordTrack Dir
    QDir currentDir = QDir::current();

    if (currentDir.cd(Common::getCommonHandle()->m_pluginPath))
        if (!currentDir.cd("doc"))
            currentDir.mkdir("doc");
}

void HandGuiding::loadTranslator(const QString &language)
{
    m_translator->load(
                QString(":/Translations/Translations/%1_template_plugin.qm")
                .arg(language));

    qDebug() << "language : "
             << QString(":/Translations/Translations/%1_template_plugin.qm")
                .arg(language);

    qApp->installTranslator(m_translator);
}

bool HandGuiding::add(
        const double &parm1, const double &parm2, double &result)
{
    return ArithmeticInterface::getArithmeticInterfaceHandle()
            ->addInterface(parm1, parm2, result);
}

bool HandGuiding::sub(
        const double &parm1, const double &parm2, double &result)
{
    return ArithmeticInterface::getArithmeticInterfaceHandle()
            ->subInterface(parm1, parm2, result);
}

bool HandGuiding::mul(
        const double &parm1, const double &parm2, double &result)
{
    return ArithmeticInterface::getArithmeticInterfaceHandle()
            ->mulInterface(parm1, parm2, result);
}

bool HandGuiding::div(
        const double &parm1, const double &parm2, double &result)
{
    return ArithmeticInterface::getArithmeticInterfaceHandle()
            ->divInterface(parm1, parm2, result);
}

bool HandGuiding::takePhoto()
{
    m_waypointList.append("0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000");
    m_waypointList.append("-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008");
    m_waypointList.append("0.227435, -0.034996, -1.237559, 0.368232, -1.570798, 0.227429");
    m_waypointList.append("-0.331342, -0.171155, -1.357385, 0.384567, -1.570794, -0.331347");

    return true;
}

bool HandGuiding::takeFirstWaypoint(QString &firstWaypoint)
{
    bool ret = true;
    if ((ret = !m_waypointList.isEmpty()))
        firstWaypoint = m_waypointList.takeFirst();

    return ret;
}
