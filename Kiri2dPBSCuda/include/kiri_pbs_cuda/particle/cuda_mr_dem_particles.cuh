/*
 * @Author: Xu.WANG
 * @Date: 2021-02-04 12:36:10
 * @LastEditTime: 2021-09-13 19:00:15
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\particle\cuda_mr_dem_particles.cuh
 */

#ifndef _CUDA_MRDEM_PARTICLES_CUH_
#define _CUDA_MRDEM_PARTICLES_CUH_

#pragma once

#include <kiri_pbs_cuda/particle/cuda_dem_particles.cuh>

namespace KIRI
{
	class CudaMRDemParticles : public CudaDemParticles
	{
	public:
		explicit CudaMRDemParticles::CudaMRDemParticles(
			const uint numOfMaxParticles)
			: CudaDemParticles(numOfMaxParticles),
			  mRadius(numOfMaxParticles)
		{
		}

		explicit CudaMRDemParticles::CudaMRDemParticles(
			const Vec_Float2 &p,
			const Vec_Float3 &col,
			const Vec_Float &radius,
			const Vec_Float &mass)
			: CudaDemParticles(p, col, mass),
			  mRadius(p.size())
		{
			KIRI_CUCALL(cudaMemcpy(mRadius.Data(), &radius[0], sizeof(float) * radius.size(), cudaMemcpyHostToDevice));
		}
		virtual ~CudaMRDemParticles()  {}
		CudaMRDemParticles(const CudaMRDemParticles &) = delete;
		CudaMRDemParticles &operator=(const CudaMRDemParticles &) = delete;

		inline float *GetRadiusPtr() const { return mRadius.Data(); }

	protected:
		CudaArray<float> mRadius;
	};

	typedef SharedPtr<CudaMRDemParticles> CudaMRDemParticlesPtr;
} // namespace KIRI

#endif