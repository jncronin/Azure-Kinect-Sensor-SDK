// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// This library
#include <k4a/k4aversion.h>
#include <k4ainternal/depth.h>

// System dependencies
#include <assert.h>
#include <stdbool.h>

// Dependent libraries
#include <k4ainternal/common.h>
#include <k4ainternal/dewrapper.h>

// System dependencies
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEPTH_CALIBRATION_DATA_SIZE 2000000

static k4a_version_t g_min_fw_version_rgb = { 1, 5, 92 };                  // 1.5.92
static k4a_version_t g_min_fw_version_depth = { 1, 5, 66 };                // 1.5.66
static k4a_version_t g_min_fw_version_audio = { 1, 5, 14 };                // 1.5.14
static k4a_version_t g_min_fw_version_depth_config = { 5006, 27, 0 };      // 5006.27 (iteration is not used, set to 0)
static k4a_version_t g_suggested_fw_version_rgb = { 1, 6, 110 };           // 1.6.110
static k4a_version_t g_suggested_fw_version_depth = { 1, 6, 79 };          // 1.6.79
static k4a_version_t g_suggested_fw_version_audio = { 1, 6, 14 };          // 1.6.14
static k4a_version_t g_suggested_fw_version_depth_config = { 6109, 7, 0 }; // 6109.7 (iteration is not used, set to 0)

typedef struct _depth_context_t
{
    depthmcu_t depthmcu;

    uint8_t *calibration_memory;
    size_t calibration_memory_size;
    bool calibration_init;

    bool running;
    k4a_hardware_version_t version;
    k4a_calibration_camera_t calibration;

    depth_cb_streaming_capture_t *capture_ready_cb;
    void *capture_ready_cb_context;
} depth_context_t;

K4A_DECLARE_CONTEXT(depth_t, depth_context_t);

depthmcu_stream_cb_t depth_capture_available;

static void log_device_info(depth_context_t *depth);
static void depth_stop_internal(depth_t depth_handle, bool quiet);
bool is_fw_version_compatable(const char *fw_type,
                              k4a_version_t *fw_version,
                              k4a_version_t *fw_min_version,
                              bool error);

bool is_fw_version_compatable(const char *fw_type, k4a_version_t *fw_version, k4a_version_t *fw_min_version, bool error)
{
    bool fw_version_good = k4a_is_version_greater_or_equal(fw_version, fw_min_version);

    if (!fw_version_good)
    {
        if (error)
        {
            LOG_ERROR("Firmware version for %s is %d.%d.%d is not current enough. Use %d.%d.%d or newer.",
                      fw_type,
                      fw_version->major,
                      fw_version->minor,
                      fw_version->iteration,
                      fw_min_version->major,
                      fw_min_version->minor,
                      fw_min_version->iteration);
        }
        else
        {
            LOG_WARNING("Firmware version for %s is %d.%d.%d. Consider upgrading to %d.%d.%d or newer.",
                        fw_type,
                        fw_version->major,
                        fw_version->minor,
                        fw_version->iteration,
                        fw_min_version->major,
                        fw_min_version->minor,
                        fw_min_version->iteration);
        }
    }
    return (fw_version_good);
}

k4a_result_t depth_create(depthmcu_t depthmcu,
                          calibration_t calibration_handle,
                          depth_cb_streaming_capture_t *capture_ready,
                          void *capture_ready_context,
                          depth_t *depth_handle)
{
    depth_context_t *depth = NULL;
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

    RETURN_VALUE_IF_ARG(K4A_RESULT_FAILED, depthmcu == NULL);
    RETURN_VALUE_IF_ARG(K4A_RESULT_FAILED, calibration_handle == NULL);
    RETURN_VALUE_IF_ARG(K4A_RESULT_FAILED, depth_handle == NULL);

    depth = depth_t_create(depth_handle);
    depth->depthmcu = depthmcu;
    depth->capture_ready_cb = capture_ready;
    depth->capture_ready_cb_context = capture_ready_context;

    if (K4A_SUCCEEDED(result))
    {
        depth->calibration_memory = (uint8_t *)malloc(DEPTH_CALIBRATION_DATA_SIZE);
        result = K4A_RESULT_FROM_BOOL(depth->calibration_memory != NULL);
    }

    if (K4A_SUCCEEDED(result))
    {
        result = K4A_RESULT_FROM_BOOL(depthmcu_wait_is_ready(depth->depthmcu));
    }

    if (K4A_SUCCEEDED(result))
    {
        result = TRACE_CALL(depth_get_device_version(*depth_handle, &depth->version));
    }

    if (K4A_SUCCEEDED(result))
    {
        log_device_info(depth);

#ifndef K4A_MTE_VERSION
        if (!is_fw_version_compatable("RGB", &depth->version.rgb, &g_min_fw_version_rgb, true) ||
            !is_fw_version_compatable("Depth", &depth->version.depth, &g_min_fw_version_depth, true) ||
            !is_fw_version_compatable("Audio", &depth->version.audio, &g_min_fw_version_audio, true) ||
            !is_fw_version_compatable("Depth Config",
                                      &depth->version.depth_sensor,
                                      &g_min_fw_version_depth_config,
                                      true))
        {
            result = K4A_RESULT_FAILED;
        }
#endif
    }

    if (K4A_SUCCEEDED(result))
    {
        is_fw_version_compatable("RGB", &depth->version.rgb, &g_suggested_fw_version_rgb, false);
        is_fw_version_compatable("Depth", &depth->version.depth, &g_suggested_fw_version_depth, false);
        is_fw_version_compatable("Audio", &depth->version.audio, &g_suggested_fw_version_audio, false);
        is_fw_version_compatable("Depth Config",
                                 &depth->version.depth_sensor,
                                 &g_suggested_fw_version_depth_config,
                                 false);
    }

    if (K4A_SUCCEEDED(result))
    {
        result = TRACE_CALL(
            calibration_get_camera(calibration_handle, K4A_CALIBRATION_TYPE_DEPTH, &depth->calibration));
    }

    if (K4A_SUCCEEDED(result))
    {
        // SDK may have crashed last session, so call stop
        depth->running = true;
        bool quiet = true;
        depth_stop_internal(*depth_handle, quiet);
    }

    if (K4A_FAILED(result))
    {
        depth_destroy(*depth_handle);
        *depth_handle = NULL;
    }

    return result;
}

void depth_destroy(depth_t depth_handle)
{
    RETURN_VALUE_IF_HANDLE_INVALID(VOID_VALUE, depth_t, depth_handle);
    depth_context_t *depth = depth_t_get_context(depth_handle);

    bool quiet = false;
    depth_stop_internal(depth_handle, quiet);

    if (depth->calibration_memory != NULL)
    {
        free(depth->calibration_memory);
    }

    depth_t_destroy(depth_handle);
}

static void log_device_info(depth_context_t *depth)
{
    k4a_log_level_t level;

    if (logger_is_file_based())
    {
        // Log device information at a 'critical' level so that even at that no matter what level the logger
        // is set to, we will always of this critical information about version configuration and hardware
        // revision (hardware revision)
        level = K4A_LOG_LEVEL_CRITICAL;
    }
    else
    {
        // Log device information to stdout at an 'info' level so that users and maintain a simple level of debug
        // output. If the user want more detailed info in a stdout log then the verbosity needs to be turned up to info
        // or higher
        level = K4A_LOG_LEVEL_INFO;
    }

    logger_log(level, __FILE__, __LINE__, "******************** Device Info ********************");
    logger_log(level, __FILE__, __LINE__, "K4A SDK version:     %s", K4A_VERSION_STR);

    char serial_number[128];
    size_t size = sizeof(serial_number);
    if (depthmcu_get_serialnum(depth->depthmcu, serial_number, &size) == K4A_BUFFER_RESULT_SUCCEEDED)
    {
        logger_log(level, __FILE__, __LINE__, "Serial Number:       %s", serial_number);
    }

    k4a_version_t *ver = &depth->version.rgb;
    logger_log(level, __FILE__, __LINE__, "RGB Sensor Version:  %d.%d.%d", ver->major, ver->minor, ver->iteration);

    ver = &depth->version.depth;
    logger_log(level, __FILE__, __LINE__, "Depth Sensor Version:%d.%d.%d", ver->major, ver->minor, ver->iteration);

    ver = &depth->version.audio;
    logger_log(level, __FILE__, __LINE__, "Mic Array Version:   %d.%d.%d", ver->major, ver->minor, ver->iteration);

    ver = &depth->version.depth_sensor;
    logger_log(level, __FILE__, __LINE__, "Sensor Config:       %d.%d", ver->major, ver->minor);
    logger_log(level,
               __FILE__,
               __LINE__,
               "Build type:          %s",
               depth->version.firmware_build == 0 ? "Release" : "Debug");
    logger_log(level,
               __FILE__,
               __LINE__,
               "Signature type:      %s",
               depth->version.firmware_signature == K4A_FIRMWARE_SIGNATURE_MSFT ?
                   "MSFT" :
                   (depth->version.firmware_signature == K4A_FIRMWARE_SIGNATURE_TEST ? "Test" : "Unsigned"));

    logger_log(level, __FILE__, __LINE__, "****************************************************");
}

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

    printf("%i,%i,%i: %f (%f)\n", best_i, best_j, best_k, best_dist, best_err);

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

/** see documentation for depthmcu_stream_cb_t
 *
 * \related depthmcu_stream_cb_t
 */
void depth_capture_available(k4a_result_t cb_result, k4a_image_t image_raw, void *context)
{
    depth_context_t *depth = (depth_context_t *)context;
    (void)depth;

    k4a_capture_t capture_raw = NULL;

    if (K4A_SUCCEEDED(cb_result))
    {
        cb_result = TRACE_CALL(capture_create(&capture_raw));
    }

    if (K4A_SUCCEEDED(cb_result))
    {
        (void)image_raw;
        capture_set_ir_image(capture_raw, image_raw);
        capture_set_depth_image(capture_raw, image_raw);
    }

    // post-capture goes here
    
    // for testing, dump phase of middle pixel at all 3 frequencies
    float phases[3];
    float d[9];

    int x = 320;
    int y = 288;

    for(int i = 0; i < 9; i++)
    {
        d[i] = GetNFOVData(x, y, i, image_get_buffer(image_raw));
    }
    for(int i = 0; i < 3; i++)
    {
        GetPhase(&d[i * 3], &phases[i], NULL, NULL);
    }
    // Apply a fiddle factor based upon experimentation to account for time delay
    //  between imaging each column of the IR image
    phases[0] = fmodf(phases[0] - 2.7f * (float)x / 200.0f, M_PI * 2.0f);
    phases[1] = fmodf(phases[1] - 2.55f * (float)x / 200.0f, M_PI * 2.0f);
    phases[2] = fmodf(phases[2] - 1.05f * (float)x / 200.0f, M_PI * 2.0f);
    if(phases[0] < 0.0f) phases[0] += M_PI * 2.0f;
    if(phases[1] < 0.0f) phases[1] += M_PI * 2.0f;
    if(phases[2] < 0.0f) phases[2] += M_PI * 2.0f;

    printf("Phases: %f, %f, %f, dist: %f\n", phases[0], phases[1], phases[2],
        GetNFOVDistance(phases, NULL));

    

    if (depth->capture_ready_cb)
    {
        depth->capture_ready_cb(cb_result, capture_raw, depth->capture_ready_cb_context);
    }
    

    if (capture_raw)
    {
        capture_dec_ref(capture_raw);
    }
}

k4a_buffer_result_t depth_get_device_serialnum(depth_t depth_handle, char *serial_number, size_t *serial_number_size)
{
    RETURN_VALUE_IF_HANDLE_INVALID(K4A_BUFFER_RESULT_FAILED, depth_t, depth_handle);
    RETURN_VALUE_IF_ARG(K4A_BUFFER_RESULT_FAILED, serial_number_size == NULL);

    depth_context_t *depth = depth_t_get_context(depth_handle);

    return TRACE_BUFFER_CALL(depthmcu_get_serialnum(depth->depthmcu, serial_number, serial_number_size));
}

k4a_result_t depth_get_device_version(depth_t depth_handle, k4a_hardware_version_t *version)
{
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

    RETURN_VALUE_IF_HANDLE_INVALID(K4A_RESULT_FAILED, depth_t, depth_handle);
    RETURN_VALUE_IF_ARG(K4A_RESULT_FAILED, version == NULL);

    depth_context_t *depth = depth_t_get_context(depth_handle);
    depthmcu_firmware_versions_t mcu_version;

    result = TRACE_CALL(depthmcu_get_version(depth->depthmcu, &mcu_version));

    if (K4A_SUCCEEDED(result))
    {
        version->rgb.major = mcu_version.rgb_major;
        version->rgb.minor = mcu_version.rgb_minor;
        version->rgb.iteration = mcu_version.rgb_build;

        version->depth.major = mcu_version.depth_major;
        version->depth.minor = mcu_version.depth_minor;
        version->depth.iteration = mcu_version.depth_build;

        version->audio.major = mcu_version.audio_major;
        version->audio.minor = mcu_version.audio_minor;
        version->audio.iteration = mcu_version.audio_build;

        version->depth_sensor.major = mcu_version.depth_sensor_cfg_major;
        version->depth_sensor.minor = mcu_version.depth_sensor_cfg_minor;
        version->depth_sensor.iteration = 0;

        switch (mcu_version.build_config)
        {
        case 0:
            version->firmware_build = K4A_FIRMWARE_BUILD_RELEASE;
            break;
        case 1:
            version->firmware_build = K4A_FIRMWARE_BUILD_DEBUG;
            break;
        default:
            LOG_WARNING("Hardware reported unknown firmware build: %d", mcu_version.build_config);
            version->firmware_build = K4A_FIRMWARE_BUILD_DEBUG;
            break;
        }

        switch (mcu_version.signature_type)
        {
        case 0:
            version->firmware_signature = K4A_FIRMWARE_SIGNATURE_MSFT;
            break;
        case 1:
            version->firmware_signature = K4A_FIRMWARE_SIGNATURE_TEST;
            break;
        case 2:
            version->firmware_signature = K4A_FIRMWARE_SIGNATURE_UNSIGNED;
            break;
        default:
            LOG_WARNING("Hardware reported unknown signature type: %d", mcu_version.signature_type);
            version->firmware_signature = K4A_FIRMWARE_SIGNATURE_UNSIGNED;
            break;
        }
    }

    return result;
}

k4a_result_t depth_start(depth_t depth_handle, const k4a_device_configuration_t *config)
{
    RETURN_VALUE_IF_HANDLE_INVALID(K4A_RESULT_FAILED, depth_t, depth_handle);
    RETURN_VALUE_IF_ARG(K4A_RESULT_FAILED, config == NULL);

    depth_context_t *depth = depth_t_get_context(depth_handle);
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

    // Turn on depth sensor (needed for most depth operations)

    if (K4A_SUCCEEDED(result))
    {
        depth->running = true; // set to true once we know we need to call depth_stop to unwind
        result = TRACE_CALL(depthmcu_depth_set_capture_mode(depth->depthmcu, config->depth_mode));
    }

    if (K4A_SUCCEEDED(result) && depth->calibration_init == false)
    {
        // Device power must be on for this to succeed
        result = TRACE_CALL(depthmcu_get_cal(depth->depthmcu,
                                             depth->calibration_memory,
                                             DEPTH_CALIBRATION_DATA_SIZE,
                                             &depth->calibration_memory_size));
        if (K4A_SUCCEEDED(result))
        {
            depth->calibration_init = true;
        }
    }

    if (K4A_SUCCEEDED(result))
    {
        result = TRACE_CALL(depthmcu_depth_set_fps(depth->depthmcu, config->camera_fps));
    }

    if (K4A_SUCCEEDED(result))
    {
        result = TRACE_CALL(depthmcu_depth_start_streaming(depth->depthmcu, depth_capture_available, depth));
    }

    if (K4A_FAILED(result))
    {
        depth_stop(depth_handle);
    }

    return result;
}

void depth_stop(depth_t depth_handle)
{
    bool quiet = false;
    depth_stop_internal(depth_handle, quiet);
}

void depth_stop_internal(depth_t depth_handle, bool quiet)
{
    RETURN_VALUE_IF_HANDLE_INVALID(VOID_VALUE, depth_t, depth_handle);

    depth_context_t *depth = depth_t_get_context(depth_handle);

    // It is ok to call this multiple times, so no lock. Only doing it once is an optimization to not stop if the
    // sensor was never started.
    if (depth->running)
    {
        depthmcu_depth_stop_streaming(depth->depthmcu, quiet);
    }
    depth->running = false;
}

#ifdef __cplusplus
}
#endif
