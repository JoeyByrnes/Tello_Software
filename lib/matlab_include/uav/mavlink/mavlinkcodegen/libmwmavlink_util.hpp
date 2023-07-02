/* Copyright 2020 The MathWorks, Inc. */
#ifndef LIBMWMAVLINK_UTIL_HPP
#define LIBMWMAVLINK_UTIL_HPP

#if defined(BUILDING_LIBMWMAVLINKCODEGEN)
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
#define LIBMWMAVLINK_API DLL_EXPORT_SYM
#else
#define LIBMWMAVLINK_API
#endif

#endif // LIBMWMAVLINK_UTIL_HPP
