#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define KinovaController_DLLIMPORT __declspec(dllimport)
#  define KinovaController_DLLEXPORT __declspec(dllexport)
#  define KinovaController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define KinovaController_DLLIMPORT __attribute__((visibility("default")))
#    define KinovaController_DLLEXPORT __attribute__((visibility("default")))
#    define KinovaController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define KinovaController_DLLIMPORT
#    define KinovaController_DLLEXPORT
#    define KinovaController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef KinovaController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define KinovaController_DLLAPI
#  define KinovaController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef KinovaController_EXPORTS
#    define KinovaController_DLLAPI KinovaController_DLLEXPORT
#  else
#    define KinovaController_DLLAPI KinovaController_DLLIMPORT
#  endif // KinovaController_EXPORTS
#  define KinovaController_LOCAL KinovaController_DLLLOCAL
#endif // KinovaController_STATIC
