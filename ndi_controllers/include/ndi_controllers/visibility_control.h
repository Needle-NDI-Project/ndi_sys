// ndi_controllers/include/ndi_controllers/visibility_control.h
#ifndef NDI_CONTROLLERS__VISIBILITY_CONTROL_H_
#define NDI_CONTROLLERS__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NDI_CONTROLLERS_EXPORT __attribute__((dllexport))
#define NDI_CONTROLLERS_IMPORT __attribute__((dllimport))
#else
#define NDI_CONTROLLERS_EXPORT __declspec(dllexport)
#define NDI_CONTROLLERS_IMPORT __declspec(dllimport)
#endif
#ifdef NDI_CONTROLLERS_BUILDING_DLL
#define NDI_CONTROLLERS_PUBLIC NDI_CONTROLLERS_EXPORT
#else
#define NDI_CONTROLLERS_PUBLIC NDI_CONTROLLERS_IMPORT
#endif
#define NDI_CONTROLLERS_PUBLIC_TYPE NDI_CONTROLLERS_PUBLIC
#define NDI_CONTROLLERS_LOCAL
#else
#define NDI_CONTROLLERS_EXPORT __attribute__((visibility("default")))
#define NDI_CONTROLLERS_IMPORT
#if __GNUC__ >= 4
#define NDI_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define NDI_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define NDI_CONTROLLERS_PUBLIC
#define NDI_CONTROLLERS_LOCAL
#endif
#define NDI_CONTROLLERS_PUBLIC_TYPE
#endif

#endif // NDI_CONTROLLERS__VISIBILITY_CONTROL_H_