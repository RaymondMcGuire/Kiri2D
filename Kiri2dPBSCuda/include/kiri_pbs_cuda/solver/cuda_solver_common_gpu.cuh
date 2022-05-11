/*
 * @Author: Xu.WANG
 * @Date: 2020-07-04 14:48:23
 * @LastEditTime: 2021-09-03 15:08:22
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dPBSCuda\include\kiri_pbs_cuda\solver\cuda_solver_common_gpu.cuh
 */

#ifndef _CUDA_SOLVER_COMMON_GPU_CUH_
#define _CUDA_SOLVER_COMMON_GPU_CUH_

#pragma once

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>
#include <curand.h>
#include <curand_kernel.h>
namespace KIRI2D
{

    // generates a random float between 0 and 1
    static __device__ float RndFloat(curandState *globalState, int ind)
    {
        curandState local_state = globalState[ind];
        float val = curand_uniform(&local_state);
        globalState[ind] = local_state;
        return val;
    }

    static __global__ void SetUpRndGen_CUDA(curandState *state, unsigned long seed)
    {
        int id = threadIdx.x;
        curand_init(seed, id, 0, &state[id]);
    }

} // namespace KIRI2D

#endif /* _CUDA_SOLVER_COMMON_GPU_CUH_ */