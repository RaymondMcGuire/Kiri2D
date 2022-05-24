/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-12 12:49:56
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-24 09:45:47
 * @FilePath: \Kiri2D\core\include\kiri_log.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _KIRI_LOG_H_
#define _KIRI_LOG_H_

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace KIRI2D
{
    class KiriLog
    {
    public:
        static void init();

        inline static std::shared_ptr<spdlog::logger> &logger() { return mLogger; };

    private:
        static std::shared_ptr<spdlog::logger> mLogger;
    };
} // namespace KIRI2D

#define KIRI_LOG_TRACE(...) ::KIRI2D::KiriLog::logger()->trace(__VA_ARGS__)
#define KIRI_LOG_INFO(...) ::KIRI2D::KiriLog::logger()->info(__VA_ARGS__)
#define KIRI_LOG_DEBUG(...) ::KIRI2D::KiriLog::logger()->debug(__VA_ARGS__)
#define KIRI_LOG_WARN(...) ::KIRI2D::KiriLog::logger()->warn(__VA_ARGS__)
#define KIRI_LOG_ERROR(...) ::KIRI2D::KiriLog::logger()->error(__VA_ARGS__)
#define KIRI_LOG_FATAL(...) ::KIRI2D::KiriLog::logger()->fatal(__VA_ARGS__)

#endif