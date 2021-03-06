/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
/// @file	Config.h
/// @brief	A system for managing and storing variables that are of
///			general interest to the system.
#pragma once

#include <Debug/Debug.h>
#include <stdio.h>
#include <stdint.h>

// run at 400Hz on all systems
#define DEFAULT_MAIN_LOOP_RATE 400
#define MAIN_LOOP_RATE         DEFAULT_MAIN_LOOP_RATE 
#define MAIN_LOOP_SECONDS      0.0025f
#define MAIN_LOOP_MICROS       2500

class Config
{
public:
    const static uint16_t CONFIG_MAGIC   = 0x5cf9; // Submarine CFG
    const static uint16_t CONFIG_VERSION = 0x01;

#pragma pack(push,1)
    struct EEprom
    {
        uint16_t _magic;
        uint16_t _version;

        struct
        {
            uint8_t  _level;
            size_t   _buffer_size;
        }
        Debug;

        struct
        {
            uint16_t _loop_rate_hz;
        }
        Scheduler;
    };
#pragma pack(pop)

    static void load();
    static void save();

    static void dump();

    static bool loaded() { return ( ( _config._magic == Config::CONFIG_MAGIC ) && ( _config._version > 0 ) ); }

    static EEprom& mirror()
    {
        return _config;
    }

protected:
    Config() {}
    ~Config() {}

    static EEprom _config;
};

/// Template class for scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
///
template<typename T>
class ParamT // : public Config
{
public:
    ParamT(T& ref, const T& defval) : /* Config(), */ _value(ref)
    {
        if (!Config::loaded())
        {
            _value = defval;
        }
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T& () const
    {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    ParamT<T>& operator= (const T& v)
    {
        DBG_TRACE_PF();
        _value = v;
        return *this;
    }

    /// bit ops on parameters
    ///
    ParamT<T>& operator |=(const T& v)
    {
        DBG_TRACE_PF();
        _value |= v;
        return *this;
    }

    ParamT<T>& operator &=(const T& v)
    {
        DBG_TRACE_PF();
        _value &= v;
        return *this;
    }

    ParamT<T>& operator +=(const T& v)
    {
        DBG_TRACE_PF();
        _value += v;
        return *this;
    }

    ParamT<T>& operator -=(const T& v)
    {
        DBG_TRACE_PF();
        _value -= v;
        return *this;
    }

protected:
    T& _value;
};

/// Template class for non-scalar variables.
///
/// Objects of this type have a value, and can be treated in many ways as though they
/// were the value.
///
/// @tparam T			The scalar type of the variable
///
template<typename T>
class ParamV // : public Config
{
public:
    ParamV(T& ref, const T& defval) : /* Config(), */ _value(ref)
    {
        if (!Config::loaded())
        {
            _value = defval;
        }
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T& () const
    {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    ParamV<T>& operator=(const T& v)
    {
        DBG_TRACE_PF();
        _value = v;
        return *this;
    }

protected:
    T& _value;
};

#define DECLARE_FIELD(Type, Class, Field, DefaultValue) Type /*Class::*/Field(Config::mirror().Class.Field,DefaultValue);

/// Convenience macro for defining instances of the ParamT template.
///
// declare a scalar type
// _t is the base type
// _suffix is the suffix on the P* type name
#define PARAMDEF(_t, _suffix)   typedef ParamT<_t> P ## _suffix;

// declare a non-scalar type
// this is used in PMath.h
// _t is the base type
// _suffix is the suffix on the P* type name
#define PARAMDEFV(_t, _suffix)   typedef ParamV<_t> P ## _suffix;

PARAMDEF(float   , Float ) // defines PFloat
PARAMDEF(int8_t  , Int8  ) // defines PInt8
PARAMDEF(int16_t , Int16 ) // defines PInt16
PARAMDEF(int32_t , Int32 ) // defines PInt32
PARAMDEF(uint8_t , UInt8 ) // defines PUInt8
PARAMDEF(uint16_t, UInt16) // defines PUInt16
PARAMDEF(uint32_t, UInt32) // defines PUInt32
PARAMDEF(size_t  , Size  ) // defines PSize
