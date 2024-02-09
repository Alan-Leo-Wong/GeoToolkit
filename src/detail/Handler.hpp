#pragma once

#include "Config.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>

NAMESPACE_BEGIN(PROJ_NAME)
    namespace detail {
#define GEOBOX_FMT_CSTR(description, ...) description
#define GEOBOX_FMT_STR(description, ...) std::string(description)
#define GEOBOX_FMT_PRINT(description, ...) /*std::printf("%s\n", description)*/
#define GEOBOX_FMT_ARG(arg)

#if defined(GEOBOX_DISABLE_ENSURES)

#define GEOBOX_ENSURE(expr, ...) ((void)0)

#elif defined(GEOBOX_ENABLE_ENSURE_HANDLER)

        void ensureFailed(char const* function, char const* file, int line,
            char const* description);

#define GEOBOX_ENSURE(expr, description, ...)                                  \
  ((expr)                                                                      \
       ? ((void)0)                                                             \
       : ::GEOBOX::ensureFailed(GEOBOX_FUNCTION, __FILE__, __LINE__,           \
                                GEOBOX_FMT_CSTR(description, ##__VA_ARGS__)))

#else

#define GEOBOX_DEDAULT_ENSURE_FAILURE_IMPL(function, file, line, description, \
                                           ...)                                \
  do {                                                                         \
    std::printf("Assert failed in function '%s', "                             \
                "file '%s', line %d.\n",                                       \
                function, file, line);                                         \
    GEOBOX_FMT_PRINT(description, ##__VA_ARGS__);                              \
    std::abort();                                                              \
  } while (0)

#ifdef __CUDACC__
#define GEOBOX_ENSURE(expr, description, ...)                                  \
  do {                                                                         \
    if (!(expr)) {                                                             \
      std::printf("Assert failed in function '%s', file '%s', line %d.\n",     \
                  GEOBOX_FUNCTION, __FILE__, __LINE__);                        \
      std::printf("%s", description);                                          \
      /* there is no std::abort in cuda kernels, hence we just print the error \
       * message here*/                                                        \
    }                                                                          \
  } while (0)
#else
#define GEOBOX_ENSURE(expr, ...)                                               \
  do {                                                                         \
    if (!(expr)) {                                                             \
      GEOBOX_DEDAULT_ENSURE_FAILURE_IMPL(GEOBOX_FUNCTION, __FILE__, __LINE__,  \
                                         ##__VA_ARGS__);                       \
    }                                                                          \
  } while (0)
#endif

#endif

    } // namespace detail
NAMESPACE_END(PROJ_NAME)