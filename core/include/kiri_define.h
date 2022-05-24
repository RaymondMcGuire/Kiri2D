/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-12 12:49:56
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-24 09:45:47
 * @FilePath: \Kiri2D\core\include\kiri_define.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _KIRI_DEFINE_H_
#define _KIRI_DEFINE_H_
#pragma once

#if defined(_WIN32) || defined(_WIN64)
#define KIRI_WINDOWS
#elif defined(__APPLE__)
#define KIRI_APPLE
#endif

#ifdef KIRI_WINDOWS
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#ifdef KIRI_APPLE
#include <stddef.h>
#include <sys/types.h>
#endif

#if defined(DEBUG) || defined(_DEBUG)
#define KIRI_DEBUG_MODE
#define RELEASE false
#define PUBLISH false
#else
#define KIRI_RELEASE_MODE
#define RELEASE true
#define PUBLISH false
#endif

#ifdef KIRI_DEBUG_MODE
#define KIRI_ENABLE_ASSERTS
#endif

// ASSERTION
#ifdef KIRI_ENABLE_ASSERTS
#ifdef KIRI_WINDOWS
#include <cassert>
#define KIRI_ASSERT(x) assert(x)
#define KIRI_STATIC_ASSERT(x, info) static_assert(x, info)
#endif
#else
#define KIRI_ASSERT(x)
#endif

#define APP_NAME "KIRI"
//#define DOUBLE_PRECISION

#endif // _KIRI_DEFINE_H_