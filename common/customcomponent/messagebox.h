#pragma once

#include <QMessageBox>

class MessageBox : public QMessageBox
{
    Q_OBJECT

public:
    static int information(
            QString hint,
            const StandardButton &buttonRole1 = Ok,
            const StandardButton &buttonRole2 = NoButton,
            const StandardButton &buttonRole3 = NoButton);

    static int critical(
            const QString &hint,
            const StandardButton &buttonRole1 = Ok,
            const StandardButton &buttonRole2 = NoButton,
            const StandardButton &buttonRole3 = NoButton);

    static int warning(
            const QString &hint,
            const StandardButton &buttonRole1 = Ok,
            const StandardButton &buttonRole2 = NoButton,
            const StandardButton &buttonRole3 = NoButton);

    static int question(
            const QString &hint,
            const StandardButton &buttonRole1 = Yes,
            const StandardButton &buttonRole2 = No,
            const StandardButton &buttonRole3 = NoButton);

    static QWidget *s_messageBoxParent;
};
