#pragma once

#include "extensionsystem/iplugin.h"

class QTranslator;

class HandGuiding : public ExtensionSystem::IPlugin
{
    Q_OBJECT

    Q_PLUGIN_METADATA(IID "com.qt-project.AUBO.TeachPendantPlugin"
                      FILE TP_PLUGIN_JSON_FILE_NAME)

public:
    explicit HandGuiding(QObject *parent = 0);

    bool pluginCommonInterface(
            const QString &data, QString &retValueOrError);

    /*
     * 以下四个接口分别对应示教器在线编程中的开始、暂定、继续和停止。
     * 对于所有插件的阻塞接口，阻塞阈值是1000ms，超过上限阻塞接口必须返回。
     * 原则上所有的阻塞函数要支持实时外部调用打断，ms级响应，
     * 当示教器在线编程暂定或停止时，插件中的阻塞接口越开返回越好。
     *
     * 该处的业务逻辑根据实际情况自行实现
     */
    bool initParms() {return true;}
    bool pausePlugin() {return true;}
    bool continuePlugin() {return true;}
    bool stopPlugin() {return true;}

    QWidget *createUI(QWidget *parent);
    void setPluginSharedFilePath(const QString &pluginPath);
    void setPluginMessageBoxParentWidget(QWidget *messageBoxParent);
    void loadTranslator(const QString &language);
    void enableEdit(const bool &/*enable*/) {}

private:
    QTranslator *m_translator;

    bool add(const double &parm1, const double &parm2, double &result);
    bool sub(const double &parm1, const double &parm2, double &result);
    bool mul(const double &parm1, const double &parm2, double &result);
    bool div(const double &parm1, const double &parm2, double &result);

    bool takePhoto();
    bool takeFirstWaypoint(QString &firstWaypoint);
    QStringList m_waypointList;
};
