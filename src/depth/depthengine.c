#include <k4a/k4aversion.h>
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

static inline float GetNFOVDistance(const float *restrict phases, float *err)
{
    // determined experimentally by measuring phase == 0 points
    // In MHz to allow calculations to better fit the dynamic range of floats
    const float f1 = 199000000.0f / 1000000.0f;
    const float f2 = 188000000.0f / 1000000.0f;
    const float f3 = 55300000.0f / 1000000.0f;

    // Gives ~5m max range
    const int f1n = 4;
    const int f2n = 4;
    const int f3n = 2;

    float best_err = FLT_MAX;
    float best_dist = 0.0f;

    int best_i = 0;
    int best_j = 0;
    int best_k = 0;

    // brute force algorithm as per https://medium.com/chronoptics-time-of-flight/phase-wrapping-and-its-solution-in-time-of-flight-depth-sensing-493aa8b21c42
    for(int k = 0; k < f3n; k++)
    {
        for(int j = 0; j < f2n; j++)
        {
            for(int i = 0; i < f1n; i++)
            {
                float d1 = (phases[0] + (float)i * 2.0f * M_PI) / 2.0f / f1;
                float d2 = (phases[1] + (float)j * 2.0f * M_PI) / 2.0f / f2;
                float d3 = (phases[2] + (float)k * 2.0f * M_PI) / 2.0f / f3;

                float d_mean = (d1 + d2 + d3) / 3.0f;
                float d_var = (powf(d1 - d_mean, 2.0f) + powf(d2 - d_mean, 2.0f) + powf(d3 - d_mean, 2.0f)) / 3.0f;

                //printf("%i,%i,%i: %f,%f,%f (%f)\n",
                //    i, j, k, d1, d2, d3, sq_err);
                if(d_var < best_err)
                {
                    best_err = d_var;
                    best_dist = d_mean;
                    best_i = i;
                    best_j = j;
                    best_k = k;
                }
            }
        }
    }

    if(err)
    {
        *err = best_err;
    }

    best_dist *= 300.0f / 2.0f / M_PI;      // c / 10e6 to account for freq in MHz

    (void)best_i;
    (void)best_j;
    (void)best_k;
    //printf("%i,%i,%i: %f (%f)\n", best_i, best_j, best_k, best_dist, best_err);

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
        float a = sqrtf(powf(d[0] - c, 2.0f) + powf(d[1] - c, 2.0f));
        *amplitude = a;
    }

    if(phase)
    {
        float b = atan2f(d[0] - c, d[1] - c);
        *phase = b;
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

    for(int y = 0; y < de->frame_height; y += de->ybin)
    {
        for(int x = 0; x < de->frame_width; x += de->xbin)
        {
            float phases[3];
            float offsets[3];
            float d[9];

            for(int i = 0; i < 9; i++)
            {
                d[i] = GetNFOVData(x, y, i, image_get_buffer(image_raw));
            }
            for(int i = 0; i < 3; i++)
            {
                GetPhase(&d[i * 3], &phases[i], NULL, &offsets[i]);
            }
            // Apply a fiddle factor based upon experimentation to account for time delay
            //  between imaging each column of the IR image
            phases[0] = fmodf(phases[0] - 2.7f * (float)x / 200.0f, M_PI * 2.0f);
            phases[1] = fmodf(phases[1] - 2.55f * (float)x / 200.0f, M_PI * 2.0f);
            phases[2] = fmodf(phases[2] - 1.05f * (float)x / 200.0f, M_PI * 2.0f);
            if(phases[0] < 0.0f) phases[0] += M_PI * 2.0f;
            if(phases[1] < 0.0f) phases[1] += M_PI * 2.0f;
            if(phases[2] < 0.0f) phases[2] += M_PI * 2.0f;

            float dist = GetNFOVDistance(phases, NULL);
            float irf = fabsf((offsets[0] + offsets[1] + offsets[2]) / 3.0f / dist / dist * 1000.0f);
            
            uint16_t depth_val = (uint16_t)(dist * 1000.0f); // mm distance
            uint16_t ir_val = (uint16_t)irf;
            for(int j = 0; j < de->ybin; j++)
            {
                for(int i = 0; i < de->xbin; i++)
                {
                    ir_data[x + i + (y + j) * de->frame_width] = ir_val;
                    depth_data[x + i + (y + j) * de->frame_width] = depth_val;
                }
            }

            //printf("Phases: %f, %f, %f, dist: %f\n", phases[0], phases[1], phases[2],
            //    GetNFOVDistance(phases, NULL));

        }
    }

    k4a_capture_t c;
    capture_create(&c);
    image_set_device_timestamp_usec(ir_image,
        image_get_device_timestamp_usec(image_raw));
    image_set_system_timestamp_nsec(ir_image,
        image_get_system_timestamp_nsec(image_raw));
    capture_set_ir_image(c, ir_image);
    capture_set_depth_image(c, depth_image);

    de->capture_ready_callback(cb_result, c, de->capture_ready_callback_context);
    capture_dec_ref(capture_raw);
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

                if((de->frame_width % (de->xbin * 2)) == 0)
                {
                    de->xbin *= 2;
                }
                if((de->frame_height % (de->ybin * 2)) == 0)
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
            break;

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
