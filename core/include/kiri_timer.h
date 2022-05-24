/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-12 12:49:56
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-24 09:42:15
 * @FilePath: \Kiri2D\core\include\kiri_timer.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI_TIMER_H_
#define _KIRI_TIMER_H_
#pragma once
#include <kiri_pch.h>

namespace KIRI2D
{
    class KiriTimeStep
    {
    public:
        KiriTimeStep(float time = 0.0f)
            : mTime(time)
        {
        }

        operator float() const { return mTime; }

        inline const float seconds() const { return mTime; }
        inline const float milliSeconds() const { return mTime * 1000.0f; }
        inline const float fps() const { return 1.f / mTime; }

    private:
        float mTime;
    };

    class KiriTimer
    {
    public:
        KiriTimer() : mName("Default")
        {
            restart();
        }

        explicit KiriTimer(const String &name) : mName(name)
        {
            restart();
        }

        inline void restart()
        {
            mStartTime = std::chrono::steady_clock::now();
        }

        inline double elapsed(bool restart = false)
        {
            mEndTime = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = mEndTime - mStartTime;
            if (restart)
                this->restart();
            return diff.count();
        }

        void resetLog(const String &tip = "", bool unitMs = false)
        {
            log(true, tip, unitMs, false);
        }

        /***
         * @description:
         * @param {reset:reset timer or not; unitMs: print ms / sec; tip: print extra info; kill: after print timer, kill thread or not}
         * @return {void}
         */
        void log(bool reset = false, const String &tip = "",
                 bool unitMs = true, bool kill = false)
        {
            if (unitMs)
            {
                if (tip.length() > 0)
                    KIRI_LOG_INFO("KiriTimer({0}) Time elapsed:{1} ms", tip, elapsed() * 1000.f);
                else
                    KIRI_LOG_INFO("KiriTimer({0}) Time elapsed:{1} ms", mName, elapsed() * 1000.f);
            }
            else
            {
                if (tip.length() > 0)
                    KIRI_LOG_INFO("KiriTimer({0}) Time elapsed:{1} s", tip, elapsed());
                else
                    KIRI_LOG_INFO("KiriTimer({0}) Time elapsed:{1} s", mName, elapsed());
            }

            if (reset)
                this->restart();

            if (kill)
                exit(5);
        }

    private:
        std::chrono::steady_clock::time_point mStartTime;
        std::chrono::steady_clock::time_point mEndTime;
        String mName;
    };
} // namespace KIRI2D

#endif //_KIRI_TIMER_H_