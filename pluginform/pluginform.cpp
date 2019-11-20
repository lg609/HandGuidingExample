#include "pluginform.h"
#include "ui_pluginform.h"
#include "common.h"
#include "communicationthread.h"
#include <QMessageBox>
#include "HandGuidingForm.h"
#include <QHBoxLayout>


PluginForm *PluginForm::s_pluginFormHandle = NULL;

PluginForm::PluginForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PluginForm)
{
    ui->setupUi(this);

    s_pluginFormHandle = this;

    mp_clsHandGuidingForm = new HandGuidingForm(this);

    QHBoxLayout *mp_clsHBoxLayout = new QHBoxLayout;
    mp_clsHBoxLayout->addWidget(mp_clsHandGuidingForm);
    this->setLayout(mp_clsHBoxLayout);
}

PluginForm::~PluginForm()
{
    delete ui;
}
