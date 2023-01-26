/***
 * @Author: Xu.WANG
 * @Date: 2022-05-11 17:58:42
 * @LastEditTime: 2022-05-11 17:58:43
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\demos\dem2d_scene1\include\uni_dem_example.h
 */

#ifndef _UNI_DEM_EXAMPLE_H_
#define _UNI_DEM_EXAMPLE_H_

#pragma once
#include <kiri2d/renderer/renderer.h>
#include <kiri2d_pbs.h>

namespace KIRI2D
{
    class UniDEM2DExample
    {
    public:
        explicit UniDEM2DExample()
        {
            this->init();
            this->setupParams();
        }

        virtual ~UniDEM2DExample()
        {
        }

        virtual void update();

    private:
        const float WINDOW_WIDTH = 720.f;
        const float WINDOW_HEIGHT = 720.f;
        const float PARTICLES_RENDER_SCALE = 300.f;

        float mTotalFrameTime = 0.f;
        float mRenderInterval = 1.f / 30.f;

        Vector2F mRenderOffset;
        Vector2F mWorldSize;
        KiriRect2 mBoundaryRect;

        KiriTimer mPerFrameTimer;
        CudaDemSystemPtr mSystem;
        KiriScene2DPtr mScene;
        KiriRenderer2DPtr mRenderer;

        virtual void init();
        virtual void setupParams();
        virtual void renderScene( std::vector<KiriCircle2<float>> particles);
    };

    typedef SharedPtr<UniDEM2DExample>
        UniDEM2DExamplePtr;
}

#endif