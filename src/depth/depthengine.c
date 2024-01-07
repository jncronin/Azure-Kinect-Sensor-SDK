#include <k4a/k4aversion.h>
#include <k4a/k4a_depth_standalone.h>
#include <k4ainternal/depthengine.h>
#include <k4ainternal/common.h>
#include <k4ainternal/image.h>
#include <k4ainternal/logging.h>
#include <k4ainternal/queue.h>
#include <azure_c_shared_utility/tickcounter.h>
#include <azure_c_shared_utility/threadapi.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#define DEWRAPPER_QUEUE_DEPTH ((uint32_t)2) // We should not need to store more than 1

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif


#ifdef __cplusplus
extern "C" {
#endif

static inline int GetNFOVData(int x, int y, int frame, const uint8_t *restrict image)
{
    const int frame_width = 640;
    const int frame_height = 576;
    const int frame_stride = frame_width * 8 / 5;
    int offset = ((frame + 1) % 4) * frame_stride / 4;
    int block_of_8 = x / 5;
    int line_idx = offset + block_of_8 * 8 + x % 5;
    int idx = y * frame_stride + frame * frame_height * frame_stride + line_idx;
    
    int d = (int)image[idx];
    if(d >= 64)
        d = 64 - d;
    return d;
}

static inline int GetWFOVBinnedData(int x, int y, int frame, const uint8_t *restrict image)
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

static inline float GetNFOVDistance(const float *restrict phases, float *err)
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
                float d1 = 0.734f / 2.0f / M_PI * (phases[0] + (float)i * 2.0f * M_PI) - 0.300f;
                float d2 = 0.778f / 2.0f / M_PI * (phases[1] + (float)j * 2.0f * M_PI) - 0.357f;
                float d3 = 2.866f / 2.0f / M_PI * (phases[2] + (float)k * 2.0f * M_PI) - 1.053f;

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

static inline void GetPhase(const float *restrict d, float *phase, float *amplitude, float *offset)
{
    // See https://math.stackexchange.com/questions/118526/fitting-a-sine-wave-of-known-frequency-through-three-points
    float c = (d[0] + d[2]) / 2.0f;
    if(offset)
    {
        *offset = c;
    }

    if(amplitude)
    {
        float a = sqrtf((d[0] - c) * (d[0] - c) + (d[1] - c) * (d[1] - c));
        *amplitude = a;
    }

    if(phase)
    {
        float b = atan2f(d[0] - c, d[1] - c);
        *phase = b;
    }
}

void CPURunNFOVUnbinnedCalculation(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin, int ybin)
{
    const int frame_width = 640;
    const int frame_height = 576;

    for(int y = 0; y < frame_height; y += ybin)
    {
        for(int x = 0; x < frame_width; x += xbin)
        {
            float phases[3];
            float offsets[3];
            float amplitudes[3];
            float d[9];

            for(int i = 0; i < 9; i++)
            {
                d[i] = GetNFOVData(x, y, i, data);
            }
            for(int i = 0; i < 3; i++)
            {
                GetPhase(&d[i * 3], &phases[i], &amplitudes[i], &offsets[i]);
            }
            // Apply a fiddle factor based upon experimentation to account for time delay
            //  between imaging each column of the IR image
            phases[0] = fmodf(phases[0] - 2.7f * (float)x / 200.0f, M_PI * 2.0f);
            phases[1] = fmodf(phases[1] - 2.55f * (float)x / 200.0f, M_PI * 2.0f);
            phases[2] = fmodf(phases[2] - 1.05f * (float)x / 200.0f, M_PI * 2.0f);
            if(phases[0] < 0.0f) phases[0] += M_PI * 2.0f;
            if(phases[1] < 0.0f) phases[1] += M_PI * 2.0f;
            if(phases[2] < 0.0f) phases[2] += M_PI * 2.0f;

            float err;
            float dist = GetNFOVDistance(phases, &err);
            //float dist = 1.0f;
            float irf = fabsf((offsets[0] + offsets[1] + offsets[2]) / 3.0f / dist / dist * 1000.0f);
            
            uint16_t depth_val = (uint16_t)(dist * 1000.0f); // mm distance
            uint16_t ir_val = (uint16_t)irf;

            // Masking calculations

            // First, NFOV uses circular lens mask
            int mask_lens = (((float)x / (float)(frame_width / 2) - 1.0f) * ((float)x / (float)(frame_width / 2) - 1.0f) +
                ((float)y / (float)(frame_height / 2) - 1.0f) * ((float)y / (float)(frame_height / 2) - 1.0f)) < 1.0f ? 1 : 0;

            // Then variance in the output values
            int mask_err = err < 0.01f ? 1 : 0;

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
                    ir_out[x + i + (y + j) * frame_width] = ir_val;
                    depth_out[x + i + (y + j) * frame_width] = depth_val;
                }
            }

            //printf("Phases: %f, %f, %f, dist: %f\n", phases[0], phases[1], phases[2],
            //    GetNFOVDistance(phases, NULL));

        }
    }

    // dump phase of mid-left, middle and mid-right pixel (for calibration purposes)
    for(int i = 0; i < 3; i++)
    {
        int x = 160 + 160 * i;
        int y = 288;

        float phases[3];
        float offsets[3];
        float d[9];

        for(int i = 0; i < 9; i++)
        {
            d[i] = GetNFOVData(x, y, i, data);
        }
        for(int i = 0; i < 3; i++)
        {
            GetPhase(&d[i * 3], &phases[i], NULL, &offsets[i]);
        }
        phases[0] = fmodf(phases[0], M_PI * 2.0f);
        phases[1] = fmodf(phases[1], M_PI * 2.0f);
        phases[2] = fmodf(phases[2], M_PI * 2.0f);
        if(phases[0] < 0.0f) phases[0] += M_PI * 2.0f;
        if(phases[1] < 0.0f) phases[1] += M_PI * 2.0f;
        if(phases[2] < 0.0f) phases[2] += M_PI * 2.0f;

        printf("(%i) P1: %f, P2: %f, P3: %f\n", i, phases[0], phases[1], phases[2]);
    }
}

void CPURunWFOVBinnedPhaseDump(unsigned short int* depth_out,
    unsigned short int* ir_out,
    const unsigned char* data,
    int xbin, int ybin)
{
    (void)depth_out;
    (void)ir_out;
    (void)xbin;
    (void)ybin;

    // dump phase of mid-left, middle and mid-right pixel (for calibration purposes)
    for(int i = 0; i < 3; i++)
    {
        int x = 128 + 128 * i;
        int y = 256;

        float phases[3];
        float offsets[3];
        float d[9];

        for(int j = 0; j < 9; j++)
        {
            d[j] = GetWFOVBinnedData(x, y, j, data);
        }
        for(int j = 0; j < 3; j++)
        {
            GetPhase(&d[j * 3], &phases[j], NULL, &offsets[j]);
        }
        phases[0] = fmodf(phases[0], M_PI * 2.0f);
        phases[1] = fmodf(phases[1], M_PI * 2.0f);
        phases[2] = fmodf(phases[2], M_PI * 2.0f);
        if(phases[0] < 0.0f) phases[0] += M_PI * 2.0f;
        if(phases[1] < 0.0f) phases[1] += M_PI * 2.0f;
        if(phases[2] < 0.0f) phases[2] += M_PI * 2.0f;

        printf("(%i) P1: %f, P2: %f, P3: %f\n", i, phases[0], phases[1], phases[2]);
    }
}

void depthengine_process_frame(k4a_capture_t capture_raw, const depthengine_t *de)
{
    k4a_image_t image_raw = capture_get_ir_image(capture_raw);

    // create output images
    k4a_image_t ir_image, depth_image;
    k4a_result_t cb_result = image_create(K4A_IMAGE_FORMAT_DEPTH16,
        de->frame_width, de->frame_height, de->frame_width * 2, ALLOCATION_SOURCE_USER, &depth_image);
    if(!K4A_SUCCEEDED(cb_result))
    {
        LOG_ERROR("Could not create output depth image %d", cb_result);
    };

    cb_result = image_create(K4A_IMAGE_FORMAT_IR16,
        de->frame_width, de->frame_height, de->frame_width * 2, ALLOCATION_SOURCE_USER, &ir_image);
    if(!K4A_SUCCEEDED(cb_result))
    {
        LOG_ERROR("Could not create ir output depth image %d", cb_result);
    }

    /* This is what I think is happening, based upon
    https://www.reddit.com/r/computervision/comments/wh8bhu/questions_about_the_operating_principle_of_the/
    
        The IR camera captures 9 images per frame.  The images are offset on the x-axis in the order
        (for NFOV unbinned): 1/4, 1/2, 3/4
            0/4, 1/4, 2/4
            3/4, 0/4, 1/4
        I am not sure why but they probably just represent the phase differences seen below

        The data format is also odd.  In every 8 byte block, the first 5 seem to be useful data,
        then there are an additional 2 1/2 bytes (i.e. 20 bits) that do vary with the image but
        I cannot find out exactly how.
        Anyhow, for NFOV unbinned the image stride is 1024 pixels, and 1024 * 5/8 = 640 which
        seems to fit.

        Also, the results are better if all of those values >= 64 are treated as negative,
         but an odd negative (not 1- or 2-complement).  We use the transform if(d>= 64) d = 64 - d
    
        During each set of three images, the emitted NIR light is amplitude modulated, and the captures
        occur at 90degree phase offsets.  Thus for each set of three points we can fit a sine wave.
        
        The three sets have the emitted NIR flashing at a different frequency, thus range ambiguity
        can be solved by finding the result of (phase + x * 2 pi) for each 3 where they all agree
        the closest (up to the greatest common demoninator of the three frequencies).
        
        There is a MS implementation but it is patented.  Therefore use a brute-force approach instead. */
    
    uint16_t *ir_data = (uint16_t *)image_get_buffer(ir_image);
    uint16_t *depth_data = (uint16_t *)image_get_buffer(depth_image);

    switch(de->dmode)
    {
        case K4A_DEPTH_MODE_NFOV_UNBINNED:
            //CPURunNFOVUnbinnedCalculation(depth_data, ir_data, image_get_buffer(image_raw), de->xbin, de->ybin);
            RunNFOVUnbinnedCalculation(depth_data, ir_data, image_get_buffer(image_raw), de->xbin, de->ybin);
            break;

        case K4A_DEPTH_MODE_WFOV_2X2BINNED:
#ifndef NDEBUG
            CPURunWFOVBinnedPhaseDump(depth_data, ir_data, image_get_buffer(image_raw), de->xbin, de->ybin);
#endif
            RunWFOVBinnedCalculation(depth_data, ir_data, image_get_buffer(image_raw), de->xbin, de->ybin);
            break;

        default:
            break;
    }
    
    k4a_capture_t c;
    capture_create(&c);
    image_set_device_timestamp_usec(ir_image,
        image_get_device_timestamp_usec(image_raw));
    image_set_system_timestamp_nsec(ir_image,
        image_get_system_timestamp_nsec(image_raw));
    image_set_device_timestamp_usec(depth_image,
        image_get_device_timestamp_usec(image_raw));
    image_set_system_timestamp_nsec(depth_image,
        image_get_system_timestamp_nsec(image_raw));
    capture_set_ir_image(c, ir_image);
    capture_set_depth_image(c, depth_image);

    de->capture_ready_callback(cb_result, c, de->capture_ready_callback_context);
    capture_dec_ref(capture_raw);
    capture_dec_ref(c);
    image_dec_ref(ir_image);
    image_dec_ref(depth_image);
    image_dec_ref(image_raw);
}

void depthengine_enqueue_frame(k4a_capture_t f, const depthengine_t *de)
{
    queue_push(de->queue, f);
}

static int depthengine_thread(void *param)
{
    depthengine_t *de = (depthengine_t *)param;
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

    while(result != K4A_RESULT_FAILED && de->thread_stop == 0)
    {
        k4a_capture_t capture_raw;

        k4a_wait_result_t wresult = queue_pop(de->queue, K4A_WAIT_INFINITE, &capture_raw);
        if (wresult != K4A_WAIT_RESULT_SUCCEEDED)
        {
            result = K4A_RESULT_FAILED;
        }

        if(K4A_SUCCEEDED(result))
        {
            tickcounter_ms_t start_time = 0;
            tickcounter_ms_t stop_time = 0;

            tickcounter_get_current_ms(de->tick, &start_time);
            depthengine_process_frame(capture_raw, de);
            tickcounter_get_current_ms(de->tick, &stop_time);

            if ((stop_time - start_time) > (unsigned)de->depth_engine_max_compute_time_ms)
            {
                LOG_WARNING("Depth image processing is too slow at %lldms, xbin=%d, ybin=%d (this may be transient).",
                            stop_time - start_time, de->xbin, de->ybin);

                if((de->frame_width % (de->xbin * 2)) == 0 && de->xbin < 8)
                {
                    de->xbin *= 2;
                }
                if((de->frame_height % (de->ybin * 2)) == 0 && de->ybin < 8)
                {
                    de->ybin *= 2;
                }
            }
            else if((stop_time - start_time) < (unsigned)de->depth_engine_max_compute_time_ms / 4)
            {
                // can speed up - needs to be <1/4 time to build in some hysteresis
                if(de->xbin > 1 && de->xbin > de->ybin)
                {
                    de->xbin /= 2;
                }
                else if(de->ybin > 1)
                {
                    de->ybin /= 2;
                }
            }
        }
    }

    return result;
}

void depthengine_start(depthengine_t *de, const k4a_device_configuration_t *config)
{
    de->dmode = config->depth_mode;

    switch(config->depth_mode)
    {
        case K4A_DEPTH_MODE_NFOV_UNBINNED:
            de->frame_width = 640;
            de->frame_height = 576;
            InitNFOVUnbinnedCalculation();
            break;

        case K4A_DEPTH_MODE_WFOV_2X2BINNED:
            de->frame_width = 512;
            de->frame_height = 512;
            InitWFOVBinnedCalculation();
            break;

        case K4A_DEPTH_MODE_WFOV_2X2BINNED_UNPROCESSED:
        case K4A_DEPTH_MODE_NFOV_UNBINNED_UNPROCESSED:
            // do nothing
            return;

        default:
            LOG_ERROR("Unsupported depth_mode %d", config->depth_mode);
            break;
    }

    de->thread_stop = 0;

    /* Use at most half the available frame time for processing,
        Increase bins at the expense of detail to improve speed */
    switch(config->camera_fps)
    {
        case K4A_FRAMES_PER_SECOND_5:
            de->depth_engine_max_compute_time_ms = 1000 / 5 / 2;
            break;
        case K4A_FRAMES_PER_SECOND_15:
            de->depth_engine_max_compute_time_ms = 1000 / 15 / 2;
            break;
        case K4A_FRAMES_PER_SECOND_30:
            de->depth_engine_max_compute_time_ms = 1000 / 30 / 2;
            break;
    }
    de->overruns = 0;
    de->xbin = 8;
    de->ybin = 8;

    queue_enable(de->queue);

    ThreadAPI_Create(&de->thread, depthengine_thread, de);
}

void depthengine_create(depthengine_t *de, depthengine_cb_streaming_capture_t *capture_ready_cb,
    void *capture_ready_context)
{
    queue_create(DEWRAPPER_QUEUE_DEPTH, "depthengine", &de->queue);
    de->tick = tickcounter_create();
    de->capture_ready_callback = capture_ready_cb;
    de->capture_ready_callback_context = capture_ready_context;
}

void depthengine_stop(depthengine_t *de)
{
    switch(de->dmode)
    {
        case K4A_DEPTH_MODE_WFOV_2X2BINNED_UNPROCESSED:
        case K4A_DEPTH_MODE_NFOV_UNBINNED_UNPROCESSED:
            return;
        default:
            break;
    }

    if(de->queue)
    {
        queue_stop(de->queue);
    }
    if(de->thread)
    {
        int result;
        ThreadAPI_Join(de->thread, &result);
        (void)result;
        de->thread = NULL;
    }
    switch(de->dmode)
    {
        case K4A_DEPTH_MODE_NFOV_UNBINNED:
            DeinitNFOVUnbinnedCalculation();
            break;

        case K4A_DEPTH_MODE_WFOV_2X2BINNED:
            DeinitWFOVBinnedCalculation();
            break;

        default:
            break;
    }
}

void depthengine_destroy(depthengine_t *de)
{
    depthengine_stop(de);
    if(de->queue)
    {
        queue_destroy(de->queue);
    }
    if(de->tick)
    {
        tickcounter_destroy(de->tick);
    }
}

#ifdef __cplusplus
}
#endif
