// ndi_driver/include/ndi_driver/visibility_control.h

#ifndef NDI_DRIVER__VISIBILITY_CONTROL_H_
#define NDI_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed from the examples on the gcc wiki:
// https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NDI_DRIVER_EXPORT __attribute__((dllexport))
#define NDI_DRIVER_IMPORT __attribute__((dllimport))
#else
#define NDI_DRIVER_EXPORT __declspec(dllexport)
#define NDI_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef NDI_DRIVER_BUILDING_DLL
#define NDI_DRIVER_PUBLIC NDI_DRIVER_EXPORT
#else
#define NDI_DRIVER_PUBLIC NDI_DRIVER_IMPORT
#endif
#define NDI_DRIVER_PUBLIC_TYPE NDI_DRIVER_PUBLIC
#define NDI_DRIVER_LOCAL
#else
#define NDI_DRIVER_EXPORT __attribute__((visibility("default")))
#define NDI_DRIVER_IMPORT
#if __GNUC__ >= 4
#define NDI_DRIVER_PUBLIC __attribute__((visibility("default")))
#define NDI_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define NDI_DRIVER_PUBLIC
#define NDI_DRIVER_LOCAL
#endif
#define NDI_DRIVER_PUBLIC_TYPE
#endif

#endif // NDI_DRIVER__VISIBILITY_CONTROL_H_