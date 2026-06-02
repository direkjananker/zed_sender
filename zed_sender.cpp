/*
 * zed_sender.cpp
 *
 * Leest stereo beelden van de ZED Mini en stuurt ze via GStreamer
 * als H.264 gecomprimeerde RTP stream over UDP naar de Quest.
 *
 * Pipeline per oog:
 *   ZED frame → OpenCV (BGRA→BGR) → appsrc → videoconvert → x264enc → rtph264pay → udpsink
 *
 * Compileren:
 *   mkdir build && cd build
 *   cmake .. && make -j4
 *
 * Draaien:
 *   ./zed_sender
 *
 * Quest ontvangt op:
 *   Links oog:  UDP poort 5600
 *   Rechts oog: UDP poort 5601
 */

#include <sl/Camera.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

// ---------------------------------------------------------------------------
// Configuratie
// ---------------------------------------------------------------------------

static constexpr const char* QUEST_IP   = "192.168.2.51"; // IP van de test  ontvanger
static constexpr int PORT_LEFT          = 5600;
static constexpr int PORT_RIGHT         = 5601;
static constexpr int VIDEO_WIDTH        = 1280;
static constexpr int VIDEO_HEIGHT       = 720;
static constexpr int TARGET_FPS         = 30;

// ---------------------------------------------------------------------------
// Latency-marker
// ---------------------------------------------------------------------------
// Tekent een rij zwart/witte blokjes linksboven in het beeld die een waarde
// (probe_id / frame-teller) coderen. De marker is gewoon BEELD, dus hij
// overleeft H.264-encoding en RTP. De Quest leest na het renderen deze paar
// pixels terug en kent zo het probe_id van het getoonde frame -> latency.
//
// Codering (moet exact matchen met de decoder, zowel marker_verify.py als de
// Quest-readback):
//   - blokgrootte MARK_BLOCK px (>16 zodat het >= 1 macroblok is en niet wordt
//     weggeveegd door compressie)
//   - blok 0      : SYNC, altijd WIT  (validatie dat de marker aanwezig is)
//   - blok 1..N   : bits van de waarde, MSB eerst, WIT=1 ZWART=0
//   - rij linksboven, y = 0 .. MARK_BLOCK
static constexpr int MARK_BLOCK = 24;   // px per blok
static constexpr int MARK_BITS  = 16;   // 16-bit waarde (0..65535, wrapt)

static void draw_marker(cv::Mat& frame, uint32_t value) {
    auto block = [&](int idx, bool white) {
        int x = idx * MARK_BLOCK;
        cv::rectangle(frame,
                      cv::Rect(x, 0, MARK_BLOCK, MARK_BLOCK),
                      white ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 0),
                      cv::FILLED);
    };
    block(0, true);  // sync-blok
    for (int b = 0; b < MARK_BITS; ++b) {
        bool bit = (value >> (MARK_BITS - 1 - b)) & 0x1u;  // MSB eerst
        block(1 + b, bit);
    }
}


// ---------------------------------------------------------------------------
// GStreamer pipeline per oog
// ---------------------------------------------------------------------------

struct EyePipeline {
    GstElement*   pipeline  = nullptr;
    GstElement*   appsrc    = nullptr;
    GstClockTime  timestamp = 0;
    const char*   name      = nullptr;
};

static bool create_pipeline(EyePipeline& ep, const char* quest_ip, int port) {
    std::string pipeline_str =
        std::string("appsrc name=src is-live=true block=true format=time") +
        " caps=video/x-raw,format=BGR,width=" + std::to_string(VIDEO_WIDTH) +
        ",height=" + std::to_string(VIDEO_HEIGHT) +
        ",framerate=" + std::to_string(TARGET_FPS) + "/1" +
        " ! videoconvert" +
        " ! video/x-raw,format=I420" +
        " ! x264enc tune=zerolatency bitrate=4000 key-int-max=30 speed-preset=ultrafast byte-stream=true aud=true insert-vui=true option-string=keyint=30:min-keyint=30"
        " ! h264parse config-interval=1"
        " ! rtph264pay config-interval=1 aggregate-mode=zero-latency pt=96" +
        " ! udpsink host=" + quest_ip + " port=" + std::to_string(port);

    GError* error = nullptr;
    ep.pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error) {
        std::cerr << "[" << ep.name << "] Pipeline fout: " << error->message << "\n";
        g_error_free(error);
        return false;
    }

    ep.appsrc = gst_bin_get_by_name(GST_BIN(ep.pipeline), "src");
    if (!ep.appsrc) {
        std::cerr << "[" << ep.name << "] appsrc niet gevonden\n";
        return false;
    }

    gst_element_set_state(ep.pipeline, GST_STATE_PLAYING);
    std::cout << "[" << ep.name << "] Pipeline gestart → " << quest_ip << ":" << port << "\n";
    return true;
}

static void push_frame(EyePipeline& ep, const cv::Mat& frame) {
    gsize size = frame.total() * frame.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame.data, size);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer)      = ep.timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, TARGET_FPS);
    ep.timestamp += GST_BUFFER_DURATION(buffer);

    GstFlowReturn ret;
    g_signal_emit_by_name(ep.appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
        std::cerr << "[" << ep.name << "] push-buffer fout: " << ret << "\n";
    }
}

static void destroy_pipeline(EyePipeline& ep) {
    if (ep.pipeline) {
        gst_element_set_state(ep.pipeline, GST_STATE_NULL);
        gst_object_unref(ep.pipeline);
        ep.pipeline = nullptr;
    }
    if (ep.appsrc) {
        gst_object_unref(ep.appsrc);
        ep.appsrc = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps        = TARGET_FPS;
    init_params.depth_mode        = sl::DEPTH_MODE::NONE;
    init_params.sdk_verbose       = false;

    std::cout << "[sender] ZED Mini openen...\n";
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "[sender] ZED open() mislukt: " << sl::toString(err) << "\n";
        return 1;
    }
    std::cout << "[sender] ZED Mini OK (" << VIDEO_WIDTH << "x" << VIDEO_HEIGHT
              << " @ " << TARGET_FPS << "fps)\n";

    sl::Mat zed_left, zed_right;

    EyePipeline left_pipe, right_pipe;
    left_pipe.name  = "LEFT";
    right_pipe.name = "RIGHT";

    if (!create_pipeline(left_pipe,  QUEST_IP, PORT_LEFT) ||
        !create_pipeline(right_pipe, QUEST_IP, PORT_RIGHT)) {
        zed.close();
        return 1;
    }

    std::cout << "[sender] Streaming gestart naar " << QUEST_IP << "\n";
    std::cout << "[sender] Links: UDP:" << PORT_LEFT << "  Rechts: UDP:" << PORT_RIGHT << "\n";
    std::cout << "[sender] Ctrl+C om te stoppen.\n";

    const auto frame_duration = std::chrono::microseconds(1000000 / TARGET_FPS);

    while (true) {
        auto frame_start = std::chrono::steady_clock::now();

        if (zed.grab() != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "[sender] grab() mislukt, overslaan...\n";
            continue;
        }

        zed.retrieveImage(zed_left,  sl::VIEW::LEFT,  sl::MEM::CPU,
                          sl::Resolution(VIDEO_WIDTH, VIDEO_HEIGHT));
        zed.retrieveImage(zed_right, sl::VIEW::RIGHT, sl::MEM::CPU,
                          sl::Resolution(VIDEO_WIDTH, VIDEO_HEIGHT));

        cv::Mat bgra_left(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC4,
                          zed_left.getPtr<sl::uchar1>());
        cv::Mat bgra_right(VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC4,
                           zed_right.getPtr<sl::uchar1>());

        cv::Mat bgr_left, bgr_right;
        cv::cvtColor(bgra_left,  bgr_left,  cv::COLOR_BGRA2BGR);
        cv::cvtColor(bgra_right, bgr_right, cv::COLOR_BGRA2BGR);

        // Latency-marker op beide ogen tekenen. Voor deze eerste test is de
        // waarde een vrij-lopende frame-teller, zodat je kunt verifiëren dat de
        // marker H.264 overleeft zonder dat de servo-handoff al nodig is.
        // Later vervangen we 'marker_value' door het probe_id uit de servo-node.
        static uint32_t marker_value = 0;
        marker_value++;
        draw_marker(bgr_left,  marker_value);
        draw_marker(bgr_right, marker_value);

        push_frame(left_pipe,  bgr_left);
        push_frame(right_pipe, bgr_right);

        auto elapsed = std::chrono::steady_clock::now() - frame_start;
        if (elapsed < frame_duration) {
            std::this_thread::sleep_for(frame_duration - elapsed);
        }
    }

    destroy_pipeline(left_pipe);
    destroy_pipeline(right_pipe);
    zed.close();
    return 0;
}