#ifndef SLAM_FACTORY_CORE_DEF_H
#define SLAM_FACTORY_CORE_DEF_H

#if defined __GNUC__
#define SF_Func __func__
#elif defined _MSC_VER
#define SF_Func __FUNCTION__
#else
#define SF_Func ""
#endif

#ifdef SFAPI_EXPORTS
#  if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#    define SF_EXPORTS __declspec(dllexport)
#  elif defined __GNUC__ && __GNUC__ >= 4
#    define SF_EXPORTS __attribute__ ((visibility ("default")))
#  endif
#endif

#ifndef SF_EXPORTS
#define SF_EXPORTS
#endif

#define SF_IN
#define SF_OUT

#ifndef SF_DEPRECATED
#  if defined(__GNUC__)
#    define SF_DEPRECATED __attribute__ ((deprecated))
#  elif defined(_MSC_VER)
#    define SF_DEPRECATED __declspec(deprecated)
#  else
#    define SF_DEPRECATED
#  endif
#endif

#ifndef SF_EXTERN_C
#  ifdef __cplusplus
#    define SF_EXTERN_C extern "C"
#  else
#    define SF_EXTERN_C
#  endif
#endif

#ifndef SF_OVERRIDE
#define SF_OVERRIDE override
#endif

#ifndef SF_FINAL
#define SF_FINAL final
#endif

#ifndef SF_CXX11
#  if __cplusplus >= 201103L || (defined _MSC_VER && _MSC_VER > 1800)
#    define SF_CXX11 1
#  endif
#else
#  if SF_CXX11 == 0
#    undef SF_CXX11
#  endif
#endif

#ifndef SF_FINAL
#  ifdef SF_CXX11
#    define SF_FINAL final
#  endif
#endif
#ifndef SF_FINAL
#  define SF_FINAL
#endif

#if defined(__cplusplus)
#  if defined(_MSC_VER) && _MSC_VER <1600

namespace sf{

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint6_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed __int64 int64_t;
typedef unsigned __int64 uint64_t;
}

#  elif defined(_MSC_VER) || __cplusplus >=201103L
#include <cstdint>

namespace sf{

using std::int8_t;
using std::uint8_t;
using std::int16_t;
using std::uint16_t;
using std::int32_t;
using std::uint32_t;
using std::int64_t;
using std::uint64_t;
}
#  else
#include <stdint.h>
namespace sf{
    
typedef ::int8_t int8_t;  //:: global representation
typedef ::uint8_t uint8_t;
typedef ::int16_t int16_t;
typedef ::uint16_t uint16_t;
typedef ::int32_t int32_t;
typedef ::uint32_t uint32_t;
typedef ::int64_t int64_t;
typedef ::uint64_t uint64_t;
}
#  endif
#else  //pure C,without CPP
#include <stdint.h>
#endif

#endif