/***
 * @Author: Xu.WANG
 * @Date: 2021-11-18 16:03:39
 * @LastEditTime: 2021-11-18 20:23:07
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI_VORO_GROUP_SITE_H_
#define _KIRI_VORO_GROUP_SITE_H_

#pragma once

#include <kiri2d/voronoi/voro_site.h>

namespace KIRI
{

    class KiriVoroGroupSite : public KiriVoroSite
    {
    public:
        explicit KiriVoroGroupSite()
            : KiriVoroSite(0.f, 0.f, 0.f) {}

        explicit KiriVoroGroupSite(float x, float y)
            : KiriVoroSite(x, y, std::numeric_limits<float>::epsilon()) {}

        explicit KiriVoroGroupSite(Vector2F pos)
            : KiriVoroSite(pos.x, pos.y, std::numeric_limits<float>::epsilon()) {}

        explicit KiriVoroGroupSite(Vector2F pos, float percent)
            : KiriVoroSite(pos.x, pos.y, std::numeric_limits<float>::epsilon(), percent) {}

        explicit KiriVoroGroupSite(float x, float y, float weight)
            : KiriVoroSite(x, y, weight) {}

        explicit KiriVoroGroupSite(Vector3F data)
            : KiriVoroSite(data) {}

        explicit KiriVoroGroupSite(Vector4F data)
            : KiriVoroSite(data) {}

        explicit KiriVoroGroupSite(float x, float y, float weight, float percent)
            : KiriVoroSite(x, y, weight, percent) {}

        virtual ~KiriVoroGroupSite() {}

        bool GetIsGroup() { return mIsGroup; }
        UInt GetGroupId() { return mGroupId; }
        Vector3F GetGroupColor() { return mGroupColor; }

        void SetGroupId(UInt id)
        {
            mGroupId = id;
            mIsGroup = true;
        }

        void SetGroupColor(Vector3F col)
        {
            mGroupColor = col;
        }

        void DisGroup()
        {
            mGroupId = -1;
            mIsGroup = false;
            mGroupColor = Vector3F(0.f);
        }

    private:
        bool mIsGroup = false;
        UInt mGroupId = -1;
        Vector3F mGroupColor = Vector3F(0.f);
    };

    typedef SharedPtr<KiriVoroGroupSite> KiriVoroGroupSitePtr;
}
#endif