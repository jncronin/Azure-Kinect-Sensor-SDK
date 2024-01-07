/* Functions to calibrate the no-de depth engine */

#include "opencv2/core.hpp"
#include <k4a/k4a.h>
#include <gtk/gtk.h>

#include <chrono>
using namespace std::chrono_literals;

#include <thread>

#include <iostream>

bool ir_image_ready = false;
bool start_capturing = false;
k4a_device_t device = NULL;

void k4a_thread_func();
std::thread k4a_thread;

uint32_t ir_image[640*576];
float ir_proc[640*576];

GtkImage *wimg;
GdkPixbuf *wpixbuf;
guchar *wpixels;
int wrowstride;

GtkLabel *lab_phase_0 = nullptr;
GtkLabel *lab_phase_1 = nullptr;
GtkLabel *lab_phase_2 = nullptr;
GtkEntry *entry_dist = nullptr;


int test_x = 320;
int test_y = 576/2;

float test_phases[3];

static void activate(GtkApplication *app, gpointer user_data);

int main(int argc, char *argv[])
{
    k4a_device_open(K4A_DEVICE_DEFAULT, &device);

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED_UNPROCESSED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;

    k4a_device_start_cameras(device, &config);

    k4a_thread = std::thread(k4a_thread_func);

    auto app = gtk_application_new("org.gtk.example", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), nullptr);
    g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);

    k4a_device_close(device);

    return 0;
}

float GetNFOVData(int x, int y, int frame, const unsigned char* image)
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

static void GetPhase(const float* d, float* phase, float* amplitude, float* offset)
{
    // See https://math.stackexchange.com/questions/118526/fitting-a-sine-wave-of-known-frequency-through-three-points
    float c = (d[0] + d[2]) / 2.0f;
    *offset = c;
    float a = sqrtf((d[0] - c) * (d[0] - c) + (d[1] - c) * (d[1] - c));
    *amplitude = a;
    float b = atan2f(d[0] - c, d[1] - c);
    *phase = b;
}

static float average_phases(const std::vector<float> &p)
{
    // average using cartesian coordinates to avoid wrap-around issues
    float tot_x = 0.0f;
    float tot_y = 0.0f;

    int n = 0;

    for(auto curp : p)
    {
        float cur_x = std::cos(curp);
        float cur_y = std::sin(curp);

        tot_x += cur_x;
        tot_y += cur_y;

        n++;
    }

    tot_x /= (float)n;
    tot_y /= (float)n;

    auto ret = std::atan2(tot_y, tot_x);
    if(ret < 0.0f)
        ret += M_PI * 2.0f;

    return ret;  
}

void k4a_thread_func()
{
    std::vector<float> p0, p1, p2;
    bool is_capturing = false;

    std::cout << "x, dist, p0, p1, p2\n";

    while(true)
    {
        k4a_capture_t last_c = nullptr;
        bool to_cont = true;

        while(to_cont)
        {
            // get most recent capture
            k4a_capture_t cur_cap;
            switch(k4a_device_get_capture(device, &cur_cap, 100))
            {
                case K4A_WAIT_RESULT_SUCCEEDED:
                    if(last_c)
                    {
                        k4a_capture_release(last_c);
                    }
                    last_c = cur_cap;
                    break;

                case K4A_WAIT_RESULT_TIMEOUT:
                    to_cont = false;
                    break;

                case K4A_WAIT_RESULT_FAILED:
                    std::cout << "k4a wait failed\n";
                    exit(-1);
                    break;
            }
        }

        if(!last_c)
        {
            continue;
        }

        k4a_image_t irraw = k4a_capture_get_ir_image(last_c);
        auto data = reinterpret_cast<unsigned char *>(k4a_image_get_buffer(irraw));

        /* Do processing */
        float min_ir = std::numeric_limits<float>::max();
        float max_ir = std::numeric_limits<float>::lowest();

        for(int y = 0; y < 576; y++)
        {
            for(int x = 0; x < 640; x++)
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

                phases[0] = fmodf(phases[0], M_PI * 2.0f);
                phases[1] = fmodf(phases[1], M_PI * 2.0f);
                phases[2] = fmodf(phases[2], M_PI * 2.0f);
                if(phases[0] < 0.0f) phases[0] += M_PI * 2.0f;
                if(phases[1] < 0.0f) phases[1] += M_PI * 2.0f;
                if(phases[2] < 0.0f) phases[2] += M_PI * 2.0f;

                if(x == test_x && y == test_y)
                {
                    test_phases[0] = phases[0];
                    test_phases[1] = phases[1];
                    test_phases[2] = phases[2];
                }

                float irf = fabsf((offsets[0] + offsets[1] + offsets[2] - amplitudes[0] - amplitudes[1] - amplitudes[2]) / 3.0f * 1000.0f);

                ir_proc[x + y * 640] = irf;
                if(irf < min_ir) min_ir = irf;
                if(irf > max_ir) max_ir = irf;
            }
        }

        k4a_image_release(irraw);
        k4a_capture_release(last_c);

        for(int y = 0; y < 576; y++)
        {
            for(int x = 0; x < 640; x++)
            {
                float scaled_ir = (ir_proc[x + 640 * y] - min_ir) / (max_ir - min_ir) * 255.0f;
                uint32_t ir_i = (uint32_t)scaled_ir;
                if(ir_i > 255) ir_i = 255;
                uint32_t cval = ir_i | (ir_i << 8) | (ir_i << 16) | 0xff000000UL;
                ir_image[x + 640 * y] = cval;

            }
        }

        // draw targets
        for(int cx = 160; cx <= 480; cx += 160)
        {
            uint32_t col = (cx == test_x) ? 0xff0000ff : 0xffff0000;
            for(int y = 576/2 - 32; y <= 576/2 + 32; y++)
            {
                for(int x = cx - 32; x <= cx + 32; x++)
                {
                    if(x == cx || y == 576/2)
                        ir_image[x + 640 * y] = col;
                }
            }
        }
        gtk_image_set_from_pixbuf(wimg, wpixbuf);

        if(lab_phase_0)
        {
            char val[32];
            snprintf(val, 31, "%.6f", test_phases[0]);
            val[31] = 0;
            gtk_label_set_text(lab_phase_0, val);
        }

        if(lab_phase_1)
        {
            char val[32];
            snprintf(val, 31, "%.6f", test_phases[1]);
            val[31] = 0;
            gtk_label_set_text(lab_phase_1, val);
        }

        if(lab_phase_2)
        {
            char val[32];
            snprintf(val, 31, "%.6f", test_phases[2]);
            val[31] = 0;
            gtk_label_set_text(lab_phase_2, val);
        }

        if(start_capturing)
        {
            is_capturing = true;
            start_capturing = false;

            p0.clear();
            p1.clear();
            p2.clear();
        }

        if(is_capturing)
        {
            p0.push_back(test_phases[0]);
            p1.push_back(test_phases[1]);
            p2.push_back(test_phases[2]);

            if(p0.size() >= 20)
            {
                float avg_p0 = average_phases(p0);
                float avg_p1 = average_phases(p1);
                float avg_p2 = average_phases(p2);

                

                std::cout << test_x << ", ";

                if(entry_dist)
                {
                    std::cout << gtk_entry_get_text(entry_dist) << ", ";
                }
                else
                {
                    std::cout << "NA, ";
                }
                
                std::cout << avg_p0 << ", " << avg_p1 << ", " << avg_p2 << "\n";

                is_capturing = false;
            }
        }
    }
}

static gboolean btn_toggle_click(GtkWidget *btn, GdkEventButton *event, gpointer data)
{
    (void)btn;
    (void)data;

    if(event->button == 1 && event->type == GDK_BUTTON_PRESS)
    {
        switch(test_x)
        {
            case 160:
                test_x = 320;
                break;

            case 320:
                test_x = 480;
                break;

            case 480:
                test_x = 160;
                break;
        }
    }

    return true;
}

static gboolean btn_sample_click(GtkWidget *btn, GdkEventButton *event, gpointer data)
{
    (void)btn;
    (void)data;

    if(event->button == 1 && event->type == GDK_BUTTON_PRESS)
    {
        start_capturing = true;
    }

    return true;
}

static void activate(GtkApplication *app, gpointer user_data)
{
    (void)user_data;

    GtkWidget *window = gtk_application_window_new(app);

    wpixbuf = gdk_pixbuf_new_from_data(reinterpret_cast<guchar *>(ir_image),
        GDK_COLORSPACE_RGB, TRUE, 8,
        640, 576, 640*4, nullptr, nullptr);
    wimg = GTK_IMAGE(gtk_image_new());

    auto box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_box_pack_start(GTK_BOX(box), GTK_WIDGET(wimg), false, false, 0);
    gtk_image_set_from_pixbuf(wimg, wpixbuf);

    auto vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);

    auto btn_toggle_target = GTK_BUTTON(gtk_button_new_with_label("toggle target"));
    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(btn_toggle_target), false, false, 5);
    auto btn_sample = GTK_BUTTON(gtk_button_new_with_label("sample"));
    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(btn_sample), false, false, 5);

    lab_phase_0 = GTK_LABEL(gtk_label_new("-"));
    lab_phase_1 = GTK_LABEL(gtk_label_new("-"));
    lab_phase_2 = GTK_LABEL(gtk_label_new("-"));

    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(lab_phase_0), false, false, 5);
    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(lab_phase_1), false, false, 5);
    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(lab_phase_2), false, false, 5);

    auto lab_dist_m = GTK_LABEL(gtk_label_new("Dist (m):"));
    entry_dist = GTK_ENTRY(gtk_entry_new());

    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(lab_dist_m), false, false, 5);
    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(entry_dist), true, false, 5);

    gtk_box_pack_start(GTK_BOX(box), GTK_WIDGET(vbox), false, false, 0);

    g_signal_connect(G_OBJECT(btn_toggle_target), "button_press_event", G_CALLBACK(btn_toggle_click), nullptr);
    g_signal_connect(G_OBJECT(btn_sample), "button_press_event", G_CALLBACK(btn_sample_click), nullptr);

    gtk_window_set_title(GTK_WINDOW(window), "Window");
    gtk_window_set_default_size(GTK_WINDOW(window), 512, 512);
    gtk_container_add(GTK_CONTAINER(window), box);
    gtk_widget_show_all(window);
}
