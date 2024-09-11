/**
 * @file macros.h
 *
 * @brief This file contains macros used in the LimX SDK.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_MACROS_H_
#define _LIMX_SDK_MACROS_H_

#if defined(_MSC_VER)
  #define LIMX_SDK_API __declspec(dllexport)
#elif __GNUC__ >= 4
  #define LIMX_SDK_API __attribute__ ((visibility("default")))
#else
  #define LIMX_SDK_API
#endif

#ifdef _MSC_VER
  #pragma warning(disable: 4251)
  #pragma warning(disable: 4275)
#endif

#endif