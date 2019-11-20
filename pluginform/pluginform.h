#pragma once

#include <QWidget>

class HandGuidingForm;


namespace Ui {
class PluginForm;
}

class PluginForm : public QWidget
{
    Q_OBJECT

public:
    explicit PluginForm(QWidget *parent = 0);
    ~PluginForm();

    static PluginForm *getPluginFormHandle() {return s_pluginFormHandle;}

private slots:

private:
    Ui::PluginForm *ui;

    static PluginForm *s_pluginFormHandle;

    HandGuidingForm *mp_clsHandGuidingForm;
};
