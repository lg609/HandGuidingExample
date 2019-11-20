#include "messagebox.h"
#include "pluginform.h"

QWidget *MessageBox::s_messageBoxParent = NULL;

int MessageBox::information(
        QString hint,
        const QMessageBox::StandardButton &buttonRole1,
        const QMessageBox::StandardButton &buttonRole2,
        const QMessageBox::StandardButton &buttonRole3)
{
    return QMessageBox::information(
                NULL != MessageBox::s_messageBoxParent?
                MessageBox::s_messageBoxParent
              : PluginForm::getPluginFormHandle(),
                QObject::tr("Information"),
                hint, buttonRole1, buttonRole2, buttonRole3);
}

int MessageBox::warning(
        const QString &hint,
        const QMessageBox::StandardButton &buttonRole1,
        const QMessageBox::StandardButton &buttonRole2,
        const QMessageBox::StandardButton &buttonRole3)
{
    return QMessageBox::warning(
                NULL != MessageBox::s_messageBoxParent?
                MessageBox::s_messageBoxParent
              : PluginForm::getPluginFormHandle(),
                QObject::tr("Warning"),
                hint, buttonRole1, buttonRole2, buttonRole3);
}

int MessageBox::critical(
        const QString &hint,
        const QMessageBox::StandardButton &buttonRole1,
        const QMessageBox::StandardButton &buttonRole2,
        const QMessageBox::StandardButton &buttonRole3)
{
    return QMessageBox::critical(
                NULL != MessageBox::s_messageBoxParent?
                MessageBox::s_messageBoxParent
              : PluginForm::getPluginFormHandle(),
                QObject::tr("Critical"),
                hint, buttonRole1, buttonRole2, buttonRole3);
}

int MessageBox::question(
        const QString &hint,
        const QMessageBox::StandardButton &buttonRole1,
        const QMessageBox::StandardButton &buttonRole2,
        const QMessageBox::StandardButton &buttonRole3)
{
    return QMessageBox::question(
                NULL != MessageBox::s_messageBoxParent?
                MessageBox::s_messageBoxParent
              : PluginForm::getPluginFormHandle(),
                QObject::tr("Question"),
                hint, buttonRole1, buttonRole2, buttonRole3);
}
