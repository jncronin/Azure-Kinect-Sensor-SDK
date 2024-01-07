
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <time.h>

#define E_PI 3.141592f

#define PROFILE 0

__device__ static void GetPhase(const float* d, float* phase, float* amplitude, float* offset)
{
    // See https://math.stackexchange.com/questions/118526/fitting-a-sine-wave-of-known-frequency-through-three-points
    float c = (d[0] + d[2]) / 2.0f;
    *offset = c;
    float a = sqrtf((d[0] - c) * (d[0] - c) + (d[1] - c) * (d[1] - c));
    *amplitude = a;
    float b = atan2f(d[0] - c, d[1] - c);
    *phase = b;
}


__device__ float GetNFOVData(int x, int y, int frame, const unsigned char* image)
{
    const int frame_width = 640;
    const int frame_height = 576;
    const int frame_stride = frame_width * 8 / 5;
    const int metadata_length = 256;
    int offset = (frame_height * frame_stride + metadata_length) * frame + metadata_length +
        y * frame_stride;
    int block_of_8 = x / 5;
    int idx = offset + block_of_8 * 8 + x % 5;

    int d = (int)image[idx];
    if (d >= 64)
        d = 64 - d;
    return d;
}

__device__ float GetWFOVBinnedData(int x, int y, int frame, const unsigned char* image)
{
    int x_offset = (160 * (frame + 1)) % 512;
    int src_x = x + x_offset;
    if (src_x >= 512) src_x -= 512;

    int pix_id = src_x + y * 512 + frame * 512 * 512;
    int block_of_8 = pix_id / 5;
    int block_id = pix_id % 5;
    int src_id = block_of_8 * 8 + block_id;

    int dval = (int)image[src_id];

    // proc
    if (dval >= 64) dval = 64 - dval;

    return dval;
}

__device__ static inline float GetNFOVDistance(const float* phases, float* err)
{
    /* Calibration gives us:
        d1 = 0.734 * phase1 - 0.300 
        d2 = 0.778 * phase2 - 0.150
        d3 = 2.866 * phase3 - 1.053
        
        For max dist of 3.86m (as per data sheet), we get
        max phase1 = 5.66 * 2pi
        max phase2 = 5.42 * 2pi
        max phase3 = 1.71 * 2pi */

    const int f1n = 5;
    const int f2n = 5;
    const int f3n = 1;

    float best_err = FLT_MAX;
    float best_dist = 0.0f;

    // brute force algorithm as per https://medium.com/chronoptics-time-of-flight/phase-wrapping-and-its-solution-in-time-of-flight-depth-sensing-493aa8b21c42
    for (int k = 0; k <= f3n; k++)
    {
        for (int j = 0; j <= f2n; j++)
        {
            for (int i = 0; i <= f1n; i++)
            {
                float d1 = 0.734f / 2.0f / E_PI * (phases[0] + (float)i * 2.0f * E_PI) - 0.300f;
                float d2 = 0.778f / 2.0f / E_PI * (phases[1] + (float)j * 2.0f * E_PI) - 0.357f;
                float d3 = 2.866f / 2.0f / E_PI * (phases[2] + (float)k * 2.0f * E_PI) - 1.053f;

                float d_mean = (d1 + d2 + d3) / 3.0f;
                float d_var = ((d1 - d_mean) * (d1 - d_mean) + (d2 - d_mean) * (d2 - d_mean) + (d3 - d_mean) * (d3 - d_mean)) / 3.0f;
                // TODO: profile to see which of these is best
#if 0
                if (d_var < best_err)
                {
                    best_err = d_var;
                    best_dist = d_mean;
#                }
#endif

#if 1
                best_dist = d_var < best_err ? d_mean : best_dist;
                best_err = d_var < best_err ? d_var : best_err;
#endif
            }
        }
    }

    *err = best_err;

    return best_dist;
}

__device__ static inline float GetWFOVBinnedDistance(const float* phases, float* err, float x)
{
    /* Calibration gives us (d in mm):
        d1 = 781.6 * phase1 + 179.73 - 4.528 * x 
        d2 = 817.8 * phase2 + 68.11 - 4.204 * x
        d3 = 3124.2 * phase3 + 538.74 - 10.39 * x
        
        For max dist of 3.86m (as per data sheet), we get
        max phase1 = 5.66 * 2pi
        max phase2 = 5.42 * 2pi
        max phase3 = 1.71 * 2pi */

    const int f1n = 5;
    const int f2n = 5;
    const int f3n = 2;

    float best_err = FLT_MAX;
    float best_dist = 0.0f;

    // brute force algorithm as per https://medium.com/chronoptics-time-of-flight/phase-wrapping-and-its-solution-in-time-of-flight-depth-sensing-493aa8b21c42
    for (int k = 0; k <= f3n; k++)
    {
        for (int j = 0; j <= f2n; j++)
        {
            for (int i = 0; i <= f1n; i++)
            {
                float d1 = 779.0995f / 2.0f / E_PI * (phases[0] + (float)i * 2.0f * E_PI) - 74.3193f - 3.3783f * x;
                float d2 = 824.3284f / 2.0f / E_PI * (phases[1] + (float)j * 2.0f * E_PI) - 93.5419f - 3.5326f * x;
                float d3 = 2702.1864f / 2.0f / E_PI * (phases[2] + (float)k * 2.0f * E_PI) - 816.6109f - 3.1806f * x;

                d1 /= 1000.0f;
                d2 /= 1000.0f;
                d3 /= 1000.0f;

                float d_mean = (d1 + d2 + d3) / 3.0f;
                float d_var = ((d1 - d_mean) * (d1 - d_mean) + (d2 - d_mean) * (d2 - d_mean) + (d3 - d_mean) * (d3 - d_mean)) / 3.0f;
                // TODO: profile to see which of these is best
#if 0
                if (d_var < best_err)
                {
                    best_err = d_var;
                    best_dist = d_mean;
                }
#endif

#if 1
                best_dist = d_var < best_err ? d_mean : best_dist;
                best_err = d_var < best_err ? d_var : best_err;
#endif
            }
        }
    }

    *err = best_err;

    return best_dist;
}


#if PROFILE
#define PROFILE_START(a) unsigned int pstart ## a, pend ## a; pstart ## a = clock();
#define PROFILE_END(a) pend ## a = clock(); dev_times ## a ## [outidx] = pend ## a - pstart ## a;
#else
#define PROFILE_START(a)
#define PROFILE_END(a)
#endif

// buffer sizes
const int NFOVUnbinned_in_count = 1024 * 576 * 9;
const int NFOVUnbinned_out_count = 640 * 576;

const int WFOVBinned_in_count = 3777232;
const int WFOVBinned_out_count = 512 * 512;


__global__ void NFOVUnbinnedKernel(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin, int ybin
#if PROFILE    
    , unsigned int *dev_times1, unsigned int *dev_times2, unsigned int *dev_times3
#endif
    )
{
    int outidx = threadIdx.x + blockIdx.x * blockDim.x;

    const int frame_width = 640;
    const int frame_height = 576;

    int x = (outidx % (frame_width / xbin)) * xbin;
    int y = (outidx / (frame_width / xbin)) * ybin;

    outidx = x + y * frame_width;

    float phases[3];
    float offsets[3];
    float amplitudes[3];
    float d[9];

    PROFILE_START(1);
    PROFILE_START(2);
    for (int i = 0; i < 9; i++)
    {
        d[i] = GetNFOVData(x, y, i, data);
    }

    for (int i = 0; i < 3; i++)
    {
        GetPhase(&d[i * 3], &phases[i], &amplitudes[i], &offsets[i]);
    }
    PROFILE_END(2);

    // Apply a fiddle factor based upon experimentation to account for time delay
    //  between imaging each column of the IR image
    phases[0] = fmodf(phases[0] - 2.7f * (float)x / 200.0f, E_PI * 2.0f);
    phases[1] = fmodf(phases[1] - 2.55f * (float)x / 200.0f, E_PI * 2.0f);
    phases[2] = fmodf(phases[2] - 1.05f * (float)x / 200.0f, E_PI * 2.0f);
    if (phases[0] < 0.0f) phases[0] += E_PI * 2.0f;
    if (phases[1] < 0.0f) phases[1] += E_PI * 2.0f;
    if (phases[2] < 0.0f) phases[2] += E_PI * 2.0f;

    PROFILE_START(3);
    float err;
    float dist = GetNFOVDistance(phases, &err);
    PROFILE_END(3);
    float irf = fabsf((offsets[0] + offsets[1] + offsets[2]) / 3.0f / dist / dist * 1000.0f);

    unsigned short int depth_val = (unsigned short int)(dist * 1000.0f); // mm distance
    unsigned short int ir_val = (unsigned short int)irf;

    // Masking calculations

    // First, NFOV uses circular lens mask
    int mask_lens = (((float)x / (float)(frame_width / 2) - 1.0f) * ((float)x / (float)(frame_width / 2) - 1.0f) +
        ((float)y / (float)(frame_height / 2) - 1.0f) * ((float)y / (float)(frame_height / 2) - 1.0f)) < 1.0f ? 1 : 0;

    // Then variance in the output values
    int mask_err = err < 0.05f ? 1 : 0;

    // Finally, assume that the amplitude of the returned signal is inversely proportional to the square of
    //  the distance
    int mask_amp = ((amplitudes[0] + amplitudes[1] + amplitudes[2]) * dist * dist) < 200.0f ? 1 : 0;

    unsigned short mask = (unsigned short)(mask_lens * mask_err * mask_amp);

    depth_val *= mask;
    ir_val *= mask_lens;

    for(int j = 0; j < ybin; j++)
    {
        for(int i = 0; i < xbin; i++)
        {
            depth_out[outidx + i + j * frame_width] = depth_val;
            ir_out[outidx + i + j * frame_width] = ir_val;
        }
    }

    PROFILE_END(1);
}

__global__ void WFOVBinnedKernel(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin, int ybin
#if PROFILE    
    , unsigned int *dev_times1, unsigned int *dev_times2, unsigned int *dev_times3
#endif
    )
{
    int outidx = threadIdx.x + blockIdx.x * blockDim.x;

    const int frame_width = 512;
    const int frame_height = 512;

    int x = (outidx % (frame_width / xbin)) * xbin;
    int y = (outidx / (frame_width / xbin)) * ybin;

    outidx = x + y * frame_width;

    float phases[3];
    float offsets[3];
    float amplitudes[3];
    float d[9];

    PROFILE_START(1);
    PROFILE_START(2);
    for (int i = 0; i < 9; i++)
    {
        d[i] = GetWFOVBinnedData(x, y, i, data);
    }

    for (int i = 0; i < 3; i++)
    {
        GetPhase(&d[i * 3], &phases[i], &amplitudes[i], &offsets[i]);
    }
    PROFILE_END(2);

    phases[0] = fmodf(phases[0], E_PI * 2.0f);
    phases[1] = fmodf(phases[1], E_PI * 2.0f);
    phases[2] = fmodf(phases[2], E_PI * 2.0f);
    if (phases[0] < 0.0f) phases[0] += E_PI * 2.0f;
    if (phases[1] < 0.0f) phases[1] += E_PI * 2.0f;
    if (phases[2] < 0.0f) phases[2] += E_PI * 2.0f;

    PROFILE_START(3);
    float err;
    float dist = GetWFOVBinnedDistance(phases, &err, x);
    PROFILE_END(3);
    float irf = fabsf((offsets[0] + offsets[1] + offsets[2]) / 3.0f / dist / dist * 1000.0f);

    unsigned short int depth_val = (unsigned short int)(dist * 1000.0f); // mm distance
    unsigned short int ir_val = (unsigned short int)irf;

    // Masking calculations

    // First, NFOV uses circular lens mask
    int mask_lens = (((float)x / (float)(frame_width / 2) - 1.0f) * ((float)x / (float)(frame_width / 2) - 1.0f) +
        ((float)y / (float)(frame_height / 2) - 1.0f) * ((float)y / (float)(frame_height / 2) - 1.0f)) < 1.0f ? 1 : 0;

    // Then variance in the output values
    int mask_err = err < 0.01f ? 1 : 0;

    // Finally, assume that the amplitude of the returned signal is inversely proportional to the square of
    //  the distance
    float amp_dist = (amplitudes[0] + amplitudes[1] + amplitudes[2]) * dist * dist;
    int mask_amp = (amp_dist >= 10.0f && amp_dist < 50.0f) ? 1 : 0;

    unsigned short mask = (unsigned short)(mask_lens * mask_err * mask_amp);

    depth_val *= mask;
    ir_val *= mask_lens;

    for(int j = 0; j < ybin; j++)
    {
        for(int i = 0; i < xbin; i++)
        {
            depth_out[outidx + i + j * frame_width] = depth_val;
            ir_out[outidx + i + j * frame_width] = ir_val;
        }
    }

    PROFILE_END(1);
}


// buffers to hold device data
unsigned char* dev_data;
unsigned short* dev_ir_out;
unsigned short* dev_depth_out;

unsigned int* dev_times1;
unsigned int* dev_times2;
unsigned int* dev_times3;

const int nthreads = 128;

extern "C" {

// Function to call the kernel
void RunNFOVUnbinnedCalculation(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin, int ybin)
{
    // TODO: add error checking here
    cudaError_t cudaStatus = cudaMemcpy(dev_data, data, NFOVUnbinned_in_count * sizeof(unsigned char), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

    NFOVUnbinnedKernel <<<NFOVUnbinned_out_count / nthreads / xbin / ybin, nthreads>>> (dev_depth_out, dev_ir_out, dev_data,
        xbin, ybin
#if PROFILE
        , dev_times1, dev_times2, dev_times3
#endif
        );

    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        return;
    }

    if(depth_out)
    {
        cudaStatus = cudaMemcpy(depth_out, dev_depth_out, NFOVUnbinned_out_count * sizeof(unsigned short), cudaMemcpyDeviceToHost);
        if (cudaStatus != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed!");
            return;
        }
    }

    if(ir_out)
    {
        cudaStatus = cudaMemcpy(ir_out, dev_ir_out, NFOVUnbinned_out_count * sizeof(unsigned short), cudaMemcpyDeviceToHost);
        if (cudaStatus != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed!");
            return;
        }
    }

#if PROFILE
    unsigned int* times1 = (unsigned int*)malloc(NFOVUnbinned_out_count * sizeof(unsigned int));
    unsigned int* times2 = (unsigned int*)malloc(NFOVUnbinned_out_count * sizeof(unsigned int));
    unsigned int* times3 = (unsigned int*)malloc(NFOVUnbinned_out_count * sizeof(unsigned int));
    cudaMemcpy(times1, dev_times1, NFOVUnbinned_out_count * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    cudaMemcpy(times2, dev_times2, NFOVUnbinned_out_count * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    cudaMemcpy(times3, dev_times3, NFOVUnbinned_out_count * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    unsigned int times1_worst = 0;
    unsigned int times2_worst = 0;
    unsigned int times3_worst = 0;

    for (int i = 0; i < NFOVUnbinned_out_count; i++)
    {
        if (times1[i] >= times1_worst) times1_worst = times1[i];
        if (times2[i] >= times2_worst) times2_worst = times2[i];
        if (times3[i] >= times3_worst) times3_worst = times3[i];
    }

    printf("t1: %i, t2: %i, t3: %i\n", times1_worst, times2_worst, times3_worst);

    free(times1);
    free(times2);
    free(times3);
#endif
}

void RunWFOVBinnedCalculation(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin, int ybin)
{
    // TODO: add error checking here
    cudaError_t cudaStatus = cudaMemcpy(dev_data, data, WFOVBinned_in_count * sizeof(unsigned char), cudaMemcpyHostToDevice);
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "cudaMemcpy failed!");
        return;
    }

    WFOVBinnedKernel <<<WFOVBinned_out_count / nthreads / xbin / ybin, nthreads>>> (dev_depth_out, dev_ir_out, dev_data,
        xbin, ybin
#if PROFILE
        , dev_times1, dev_times2, dev_times3
#endif
        );

    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
        fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
        return;
    }

    if(depth_out)
    {
        cudaStatus = cudaMemcpy(depth_out, dev_depth_out, WFOVBinned_out_count * sizeof(unsigned short), cudaMemcpyDeviceToHost);
        if (cudaStatus != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed!");
            return;
        }
    }

    if(ir_out)
    {
        cudaStatus = cudaMemcpy(ir_out, dev_ir_out, WFOVBinned_out_count * sizeof(unsigned short), cudaMemcpyDeviceToHost);
        if (cudaStatus != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed!");
            return;
        }
    }

#if PROFILE
    unsigned int* times1 = (unsigned int*)malloc(WFOVBinned_out_count * sizeof(unsigned int));
    unsigned int* times2 = (unsigned int*)malloc(WFOVBinned_out_count * sizeof(unsigned int));
    unsigned int* times3 = (unsigned int*)malloc(WFOVBinned_out_count * sizeof(unsigned int));
    cudaMemcpy(times1, dev_times1, WFOVBinned_out_count * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    cudaMemcpy(times2, dev_times2, WFOVBinned_out_count * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    cudaMemcpy(times3, dev_times3, WFOVBinned_out_count * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    unsigned int times1_worst = 0;
    unsigned int times2_worst = 0;
    unsigned int times3_worst = 0;

    for (int i = 0; i < WFOVBinned_out_count; i++)
    {
        if (times1[i] >= times1_worst) times1_worst = times1[i];
        if (times2[i] >= times2_worst) times2_worst = times2[i];
        if (times3[i] >= times3_worst) times3_worst = times3[i];
    }

    printf("t1: %i, t2: %i, t3: %i\n", times1_worst, times2_worst, times3_worst);

    free(times1);
    free(times2);
    free(times3);
#endif
}


// Init function
void InitNFOVUnbinnedCalculation()
{
    cudaSetDevice(0);
    cudaMalloc(&dev_data, NFOVUnbinned_in_count * sizeof(unsigned char));
    cudaMalloc(&dev_ir_out, NFOVUnbinned_out_count * sizeof(unsigned short int));
    cudaMalloc(&dev_depth_out, NFOVUnbinned_out_count * sizeof(unsigned short int));

#ifdef PROFILE
    cudaMalloc(&dev_times1, NFOVUnbinned_out_count * sizeof(unsigned int));
    cudaMalloc(&dev_times2, NFOVUnbinned_out_count * sizeof(unsigned int));
    cudaMalloc(&dev_times3, NFOVUnbinned_out_count * sizeof(unsigned int));
#endif
}

void InitWFOVBinnedCalculation()
{
    cudaSetDevice(0);
    cudaMalloc(&dev_data, WFOVBinned_in_count * sizeof(unsigned char));
    cudaMalloc(&dev_ir_out, WFOVBinned_out_count * sizeof(unsigned short int));
    cudaMalloc(&dev_depth_out, WFOVBinned_out_count * sizeof(unsigned short int));

#ifdef PROFILE
    cudaMalloc(&dev_times1, WFOVBinned_out_count * sizeof(unsigned int));
    cudaMalloc(&dev_times2, WFOVBinned_out_count * sizeof(unsigned int));
    cudaMalloc(&dev_times3, WFOVBinned_out_count * sizeof(unsigned int));
#endif
}


// Dealloc function
void DeinitNFOVUnbinnedCalculation()
{
    if (dev_data)
    {
        cudaFree(dev_data);
        dev_data = NULL;
    }
    if (dev_ir_out)
    {
        cudaFree(dev_ir_out);
        dev_ir_out = NULL;
    }
    if (dev_depth_out)
    {
        cudaFree(dev_depth_out);
        dev_depth_out = NULL;
    }

#if PROFILE
    if (dev_times1)
    {
        cudaFree(dev_times1);
        dev_times1 = NULL;
    }
    if (dev_times2)
    {
        cudaFree(dev_times2);
        dev_times2 = NULL;
    }
    if (dev_times3)
    {
        cudaFree(dev_times3);
        dev_times3 = NULL;
    }
#endif

}

void DeinitWFOVBinnedCalculation()
{
    if (dev_data)
    {
        cudaFree(dev_data);
        dev_data = NULL;
    }
    if (dev_ir_out)
    {
        cudaFree(dev_ir_out);
        dev_ir_out = NULL;
    }
    if (dev_depth_out)
    {
        cudaFree(dev_depth_out);
        dev_depth_out = NULL;
    }

#if PROFILE
    if (dev_times1)
    {
        cudaFree(dev_times1);
        dev_times1 = NULL;
    }
    if (dev_times2)
    {
        cudaFree(dev_times2);
        dev_times2 = NULL;
    }
    if (dev_times3)
    {
        cudaFree(dev_times3);
        dev_times3 = NULL;
    }
#endif

}


}
