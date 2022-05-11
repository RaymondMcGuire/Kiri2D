/***
 * @Author: Xu.WANG
 * @Date: 2022-05-11 12:51:39
 * @LastEditTime: 2022-05-11 18:00:13
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\core\include\kiri_log.h
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
        static void Init();

        inline static std::shared_ptr<spdlog::logger> &GetLogger() { return mLogger; };

    private:
        static std::shared_ptr<spdlog::logger> mLogger;
    };
} // namespace KIRI2D

#define KIRI_LOG_TRACE(...) ::KIRI2D::KiriLog::GetLogger()->trace(__VA_ARGS__)
#define KIRI_LOG_INFO(...) ::KIRI2D::KiriLog::GetLogger()->info(__VA_ARGS__)
#define KIRI_LOG_DEBUG(...) ::KIRI2D::KiriLog::GetLogger()->debug(__VA_ARGS__)
#define KIRI_LOG_WARN(...) ::KIRI2D::KiriLog::GetLogger()->warn(__VA_ARGS__)
#define KIRI_LOG_ERROR(...) ::KIRI2D::KiriLog::GetLogger()->error(__VA_ARGS__)
#define KIRI_LOG_FATAL(...) ::KIRI2D::KiriLog::GetLogger()->fatal(__VA_ARGS__)

#endif