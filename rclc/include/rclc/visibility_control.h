#ifndef RCLC__VISIBILITY_CONTROL_H_
#define RCLC__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCLC_EXPORT __attribute__ ((dllexport))
    #define RCLC_IMPORT __attribute__ ((dllimport))
  #else
    #define RCLC_EXPORT __declspec(dllexport)
    #define RCLC_IMPORT __declspec(dllimport)
  #endif
  #ifdef RCLC_BUILDING_DLL
    #define RCLC_PUBLIC RCLC_EXPORT
  #else
    #define RCLC_PUBLIC RCLC_IMPORT
  #endif
  #define RCLC_PUBLIC_TYPE RCLC_PUBLIC
  #define RCLC_LOCAL
#else
  #define RCLC_EXPORT __attribute__ ((visibility("default")))
  #define RCLC_IMPORT
  #if __GNUC__ >= 4
    #define RCLC_PUBLIC __attribute__ ((visibility("default")))
    #define RCLC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCLC_PUBLIC
    #define RCLC_LOCAL
  #endif
  #define RCLC_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RCLC__VISIBILITY_CONTROL_H_
