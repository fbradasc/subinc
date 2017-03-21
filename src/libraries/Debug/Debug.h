
#pragma once

#if defined(DEBUG)

#define DBG_STRCAT(a,b) a ## b

#define DBG_MSG_F(fmt, args...)    Debug::printf(Debug::MSG , fmt , ##args);
#define DBG_INF_F(fmt, args...)    Debug::printf(Debug::INF , fmt , ##args);
#define DBG_WRN_F(fmt, args...)    Debug::printf(Debug::WRN , fmt , ##args);
#define DBG_DBG_F(fmt, args...)    Debug::printf(Debug::DBG , fmt , ##args);
#define DBG_ERR_F(fmt, args...)    Debug::printf(Debug::ERR , fmt , ##args);
                              
#define DBG_MSG_T(txt)             Debug::print (Debug::MSG , txt);
#define DBG_INF_T(txt)             Debug::print (Debug::INF , txt);
#define DBG_WRN_T(txt)             Debug::print (Debug::WRN , txt);
#define DBG_DBG_T(txt)             Debug::print (Debug::DBG , txt);
#define DBG_ERR_T(txt)             Debug::print (Debug::ERR , txt);
                              
#define DBG_PRINTF(fmt, args...)   Debug::printf(Debug::ALL , fmt , ##args);
#define DBG_PRINT(txt)             Debug::print (Debug::ALL , txt);

#define DBG_TRACE(fmt, args...)    Debug::printf(Debug::TRC , "%s:%d - "      fmt "\n", __FILE__ , __LINE__ ,                       ##args);
#define DBG_TRACE_F(fmt, args...)  Debug::printf(Debug::TRC , "%s:%d - %s - " fmt "\n", __FILE__ , __LINE__ , __FUNCTION__        , ##args);
#define DBG_TRACE_PF(fmt, args...) Debug::printf(Debug::TRC , "%s:%d - %s - " fmt "\n", __FILE__ , __LINE__ , __PRETTY_FUNCTION__ , ##args);

#else

#define DBG_MSG_F(fmt, ...)
#define DBG_INF_F(fmt, ...)
#define DBG_WRN_F(fmt, ...)
#define DBG_DBG_F(fmt, ...)
#define DBG_ERR_F(fmt, ...)
                 
#define DBG_MSG_T(txt)
#define DBG_INF_T(txt)
#define DBG_WRN_T(txt)
#define DBG_DBG_T(txt)
#define DBG_ERR_T(txt)

#define DBG_PRINTF(fmt, ...)
#define DBG_PRINT(txt)

#define DBG_TRACE(fmt, ...)
#define DBG_TRACE_F(fmt, ...)
#define DBG_TRACE_PF(fmt, ...)

#endif

#if defined(DEBUG)

class Debug
{
public:
    enum LevelT
    {
        ALL = 0xff,
        MSG = 1<<0,
        INF = 1<<1,
        WRN = 1<<2,
        DBG = 1<<3,
        ERR = 1<<4,
        TRC = 1<<5,
    };

    static bool enabled(const Debug::LevelT level);

    static int printf(const Debug::LevelT level, const char *fmt, ...);
    static int print (const Debug::LevelT level, const char *txt     );

protected:
    Debug();

private:
    static int print (const char *txt);
};

#endif
