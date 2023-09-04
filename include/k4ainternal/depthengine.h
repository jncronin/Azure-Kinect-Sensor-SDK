#ifndef DEPTHENGINE_H
#define DEPTHENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <k4ainternal/queue.h>
#include <k4ainternal/image.h>
#include <k4ainternal/capture.h>
#include <azure_c_shared_utility/tickcounter.h>
#include <azure_c_shared_utility/threadapi.h>

typedef void(depthengine_cb_streaming_capture_t)(k4a_result_t result, k4a_capture_t capture_handle, void *callback_context);


typedef struct _depthengine_t
{
    queue_t queue;
    int frame_width, frame_height;
    int thread_stop;
    int depth_engine_max_compute_time_ms;
    k4a_depth_mode_t dmode;
    TICK_COUNTER_HANDLE tick;
    THREAD_HANDLE thread;
    int overruns;
    int xbin;
    int ybin;
    depthengine_cb_streaming_capture_t *capture_ready_callback;
    void *capture_ready_callback_context;

} depthengine_t;

void depthengine_process_frame(k4a_capture_t capture_raw, const depthengine_t *de);
void depthengine_create(depthengine_t *de, depthengine_cb_streaming_capture_t* capture_ready_cb,
    void *capture_ready_context);
void depthengine_destroy(depthengine_t *de);
void depthengine_start(depthengine_t *de, const k4a_device_configuration_t *config);
void depthengine_stop(depthengine_t *de);
void depthengine_enqueue_frame(k4a_capture_t f, const depthengine_t *de);


#ifdef __cplusplus
}
#endif

#endif
