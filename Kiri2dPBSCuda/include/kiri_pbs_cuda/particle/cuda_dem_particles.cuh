/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-09-13 15:48:54
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\particle\cuda_dem_particles.cuh
 */

#ifndef _CUDA_DEM_PARTICLES_CUH_
#define _CUDA_DEM_PARTICLES_CUH_

#pragma once

#include <kiri_pbs_cuda/particle/cuda_particles.cuh>

namespace KIRI2D
{
	class CudaDemParticles : public CudaParticles
	{
	public:
		explicit CudaDemParticles::CudaDemParticles(
			const uint numOfMaxParticles)
			: CudaParticles(numOfMaxParticles),
			  mVel(numOfMaxParticles),
			  mAcc(numOfMaxParticles),
			  mCol(numOfMaxParticles),
			  mMass(numOfMaxParticles)
		{
		}

		explicit CudaDemParticles::CudaDemParticles(
			const Vec_Float2 &p,
			const Vec_Float3 &col,
			const Vec_Float &mass)
			: CudaParticles(p),
			  mVel(p.size()),
			  mAcc(p.size()),
			  mCol(p.size()),
			  mMass(p.size())
		{
			KIRI_CUCALL(cudaMemcpy(mCol.Data(), &col[0], sizeof(float3) * col.size(), cudaMemcpyHostToDevice));
			KIRI_CUCALL(cudaMemcpy(mMass.Data(), &mass[0], sizeof(float) * mass.size(), cudaMemcpyHostToDevice));
		}
		virtual ~CudaDemParticles()  {}
		CudaDemParticles(const CudaDemParticles &) = delete;
		CudaDemParticles &operator=(const CudaDemParticles &) = delete;

		void Advect(const float dt, const float damping);

		inline float2 *GetVelPtr() const { return mVel.Data(); }
		inline float2 *GetAccPtr() const { return mAcc.Data(); }
		inline float3 *GetColPtr() const { return mCol.Data(); }
		inline float *GetMassPtr() const { return mMass.Data(); }

	protected:
		CudaArray<float2> mVel;
		CudaArray<float2> mAcc;
		CudaArray<float3> mCol;
		CudaArray<float> mMass;
	};

	typedef SharedPtr<CudaDemParticles> CudaDemParticlesPtr;
} // namespace KIRI2D

#endif