#include "common.h"
#include <QUuid>
#include "math.h"
#include <QSettings>
#include "messagebox.h"

Common *Common::s_commonHandle = NULL;

Common *Common::getCommonHandle()
{
    if (NULL == s_commonHandle)
        s_commonHandle = new Common;

    return s_commonHandle;
}

Common::Common()
{
    m_pluginPath.clear();
}
