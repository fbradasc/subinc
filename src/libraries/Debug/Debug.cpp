#if defined(DEBUG)

#include <Debug/Debug.h>
#include <Config/Config.h>
#include <stdio.h>
#include <stdarg.h>

DECLARE_FIELD( PUInt8, Debug, _level      , 255 );
DECLARE_FIELD( PSize , Debug, _buffer_size, 256 );

char* _buffer = nullptr;

// constructor
Debug::Debug(void)
{
}

bool Debug::enabled(const Debug::LevelT level)
{
    uint8_t curlevel = _level;

    return ( Debug::ALL == level ) || ( ( curlevel & level ) != 0 );
}

int Debug::printf(const Debug::LevelT level, const char *fmt, ...)
{
    size_t retval = -1;

    if (Debug::enabled(level))
    {
        if (nullptr == _buffer)
        {
            _buffer = new char[_buffer_size];
        }

        if (nullptr != _buffer)
        {
            va_list args;

            va_start(args, fmt);

            retval = vsnprintf(_buffer, _buffer_size, fmt, args);

            va_end(args);

            if ((retval > 0) && (retval < _buffer_size))
            {
                Debug::print(_buffer);
            }
        }
    }

    return retval;
}

int Debug::print(const Debug::LevelT level, const char *txt)
{
    int retval = -1;

    if (Debug::enabled(level))
    {
        Debug::print(txt);
    }

    return retval;
}

int Debug::print(const char *txt)
{
    return fputs(txt,stdout);
}

#endif
