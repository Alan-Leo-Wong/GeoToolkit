#pragma once


#ifdef HAVE_CONFIG_H
#include "../project_config.h"
#endif

#if defined(__clang__) || defined(__GNUC__)
#define FORCE_INLINE __attribute__((always_inline)) inline
#elif defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#endif

#ifdef __GNUC__
#define GEOBOX_FUNCTION __PRETTY_FUNCTION__
#elif defined(__clang__) || (_MSC_VER >= 1310)
#define GEOBOX_FUNCTION __FUNCTION__
#else
#define GEOBOX_FUNCTION "unknown"
#endif

/* namespace macro */
#define PROJ_NAME geobox

#if !defined(NAMESPACE_BEGIN)
#define NAMESPACE_BEGIN(name) namespace name {
#endif

#if !defined(NAMESPACE_END)
#define NAMESPACE_END(name) }
#endif
