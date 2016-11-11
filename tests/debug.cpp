#include <Debug/Debug.h>

int main(int argc, char *argv[])
{
DBG_TRACE_PF("Enter");

    DBG_MSG_F("MSG Counter %d\n", __COUNTER__);
    DBG_INF_F("INF Line %d\n"   , __LINE__   );
    DBG_WRN_F("WRN Counter %d\n", __COUNTER__);
    DBG_DBG_F("DBG Line %d\n"   , __LINE__   );
    DBG_ERR_F("ERR Counter %d\n", __COUNTER__);

DBG_TRACE();

    DBG_PRINTF("printf(%d:%x)\n", __LINE__, __COUNTER__);
    
DBG_TRACE_F();

    DBG_MSG_T("MSG\n");
    DBG_INF_T("INF\n");
    DBG_WRN_T("WRN\n");
    DBG_DBG_T("DBG\n");
    DBG_ERR_T("ERR\n");

DBG_TRACE_PF();

    DBG_PRINT("print\n");

DBG_TRACE_F("Exit");
}
