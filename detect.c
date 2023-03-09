/**
 * Copyright 2022 by Au-Zone Technologies.  All Rights Reserved.
 *
 * Software that is described herein is for illustrative purposes only which
 * provides customers with programming information regarding the DeepView VAAL
 * library. This software is supplied "AS IS" without any warranties of any
 * kind, and Au-Zone Technologies and its licensor disclaim any and all
 * warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  Au-Zone Technologies assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under
 * any patent, copyright, mask work right, or any other intellectual property
 * rights in or to any products. Au-Zone Technologies reserves the right to make
 * changes in the software without notification. Au-Zone Technologies also makes
 * no representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 */

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <vaal.h>
#include <videostream.h>
#include <zmq.h>

#include "flex.h"

#define USEC_PER_SEC 1000000ll
#define NSEC_PER_SEC (1000ll * USEC_PER_SEC)

#define array_size(x) (sizeof(x) / sizeof(*x))

struct json {
    size_t capacity;
    size_t length;
    char*  buffer;
};

static int         running = 1;
static VSLClient*  vsl     = NULL;
static struct json json    = {0};
static const char* topic   = "DETECTION";

static int64_t
clock_now()
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * NSEC_PER_SEC + now.tv_nsec;
}

static int64_t
update_fps()
{
    static int64_t previous_time   = 0;
    static int64_t fps_history[30] = {0};
    static int     fps_index       = 0;

    int64_t timestamp      = clock_now();
    int64_t frame_time     = timestamp - previous_time;
    previous_time          = timestamp;
    fps_history[fps_index] = NSEC_PER_SEC / frame_time;
    fps_index              = fps_index >= 29 ? 0 : fps_index + 1;

    int64_t fps = 0;
    for (int i = 0; i < 30; i++) { fps += fps_history[i]; }
    fps /= 30;

    return fps;
}

/**
 * On sigint we set running to 0 (stop the event loop) and disconnect the vsl
 * client socket causing outstanding vsl_frame_wait() to terminate.
 */
static void
quit(int signum)
{
    (void) signum;

    running = 0;
    vsl_client_disconnect(vsl);
}

/**
 * This function is where we read the videostream frame and do perform model
 * inferencing with VisionPack VAAL.
 */
static int
handle_vsl(void* pub, VAALContext* vaal, VAALBox* boxes, int max_boxes)
{
    int     err;
    int64_t start;
    size_t  sz  = 0;
    char*   buf = NULL;

    /**
     * The vsl_frame_wait function will block until the next frame is received.
     *
     * IMPORTANT: vsl_frame_release must be called on the VSLFrame returned by
     * this function.  Failure to do so will result in leaked file descriptors
     * and the eventual termination of the application by the operating system.
     */
    VSLFrame* frame = vsl_frame_wait(vsl, 0);
    if (!frame) { return 0; }

    /**
     * The vsl_frame_trylock will attempt to lock the frame so that it can live
     * longer than the default lifespan, typically 100ms. It is technically not
     * needed in this case as the vaal_load_frame function will complete well
     * within the default lifespan of the frame as load_frame will complete in
     * under 5ms.  The trylock is included of illustrative purposes for cases
     * where the frame could be used beyond the default 100ms lifespan.
     */
    err = vsl_frame_trylock(frame);
    if (err) {
        fprintf(stderr, "failed to lock frame: %s\n", strerror(errno));
        vsl_frame_release(frame);
        return 0;
    }

    int64_t  fps       = update_fps();
    int      dmabuf    = vsl_frame_handle(frame);
    int      width     = vsl_frame_width(frame);
    int      height    = vsl_frame_height(frame);
    uint32_t fourcc    = vsl_frame_fourcc(frame);
    int64_t  timestamp = vsl_frame_timestamp(frame);

    start = vaal_clock_now();
    err   = vaal_load_frame_dmabuf(vaal,
                                 NULL,
                                 dmabuf,
                                 fourcc,
                                 width,
                                 height,
                                 NULL,
                                 0);
    if (err) {
        fprintf(stderr,
                "failed to load frame into model: %s\n",
                vaal_strerror(err));
        vsl_frame_unlock(frame);
        vsl_frame_release(frame);
        return -1;
    }
    int64_t load_ns = vaal_clock_now() - start;

    start = vaal_clock_now();
    err   = vaal_run_model(vaal);
    if (err) {
        fprintf(stderr, "failed to run model: %s\n", vaal_strerror(err));
        vsl_frame_unlock(frame);
        vsl_frame_release(frame);
        return -1;
    }
    int64_t model_ns = vaal_clock_now() - start;

    /**
     * The vaal_boxes function will load our array of VAALBox structures with
     * the bounding boxes identified by the model output.  The max_boxes
     * parameter is the size of the boxes array while the n_boxes is updated by
     * vaal_boxes with the actual number of bounding boxes loaded into the boxes
     * array, in other words the number of box detections from this inference.
     *
     * The vaal_boxes function internally handles the model output box decoding
     * and nms.
     */
    start = vaal_clock_now();
    size_t n_boxes;
    err = vaal_boxes(vaal, boxes, max_boxes, &n_boxes);
    if (err) {
        fprintf(stderr,
                "failed to read bounding boxes from model: %s\n",
                vaal_strerror(err));
        vsl_frame_unlock(frame);
        vsl_frame_release(frame);
        return -1;
    }
    int64_t boxes_ns = vaal_clock_now() - start;

    /**
     * The following code generates a JSON structure with the inference results.
     * The model and timing information is populated into fields of the root
     * object then an array of detected boxes is populated.
     *
     * Note: we have some special handling at the front of the message for the
     * message queue subscription topic.
     */
    int topic_len = strlen(topic) + 1;

    if (json.capacity < topic_len) {
        char* ptr = realloc(json.buffer, topic_len);
        if (!ptr) {
            fprintf(stderr, "cannot grow json buffer: out of memory\n");
            return -1;
        }
        json.buffer = ptr;
    }

    char* pos = json.buffer;
    pos += snprintf(pos, json.capacity, "%s ", topic);

    pos = json_objOpen_flex(&json.buffer, &json.capacity, pos, NULL);
    pos = json_int64_flex(&json.buffer,
                          &json.capacity,
                          pos,
                          "timestamp",
                          timestamp);
    pos =
        json_int64_flex(&json.buffer, &json.capacity, pos, "load_ns", load_ns);
    pos = json_int64_flex(&json.buffer,
                          &json.capacity,
                          pos,
                          "model_ns",
                          model_ns);
    pos = json_int64_flex(&json.buffer,
                          &json.capacity,
                          pos,
                          "boxes_ns",
                          boxes_ns);
    pos = json_int64_flex(&json.buffer, &json.capacity, pos, "fps", fps);

    pos = json_arrOpen_flex(&json.buffer, &json.capacity, pos, "objects");

    for (int i = 0; i < n_boxes; i++) {
        const VAALBox* box   = &boxes[i];
        const char*    label = vaal_label(vaal, box->label);

        pos = json_objOpen_flex(&json.buffer, &json.capacity, pos, NULL);
        pos = json_objOpen_flex(&json.buffer, &json.capacity, pos, "bbox");
        pos = json_double_flex(&json.buffer,
                               &json.capacity,
                               pos,
                               "xmin",
                               box->xmin);
        pos = json_double_flex(&json.buffer,
                               &json.capacity,
                               pos,
                               "ymin",
                               box->ymin);
        pos = json_double_flex(&json.buffer,
                               &json.capacity,
                               pos,
                               "xmax",
                               box->xmax);
        pos = json_double_flex(&json.buffer,
                               &json.capacity,
                               pos,
                               "ymax",
                               box->ymax);
        pos = json_objClose_flex(&json.buffer, &json.capacity, pos);

        pos = json_double_flex(&json.buffer,
                               &json.capacity,
                               pos,
                               "score",
                               box->score);

        if (label) {
            pos = json_str_flex(&json.buffer,
                                &json.capacity,
                                pos,
                                "label",
                                label);
        }

        pos = json_objClose_flex(&json.buffer, &json.capacity, pos);
    }

    pos = json_arrClose_flex(&json.buffer, &json.capacity, pos);

    pos         = json_objClose_flex(&json.buffer, &json.capacity, pos);
    pos         = json_end(pos);
    json.length = pos - json.buffer;

    fprintf(stderr, "%.*s\n", json.length, json.buffer);

    // err = zmq_send(pub, topic, strlen(topic), ZMQ_SNDMORE);
    // if (err == -1) {
    //     fprintf(stderr, "failed to publish topic: %s\n",
    //     zmq_strerror(errno));
    // }

    err = zmq_send(pub, json.buffer, json.length, 0);
    if (err == -1) {
        fprintf(stderr, "failed to publish result: %s\n", zmq_strerror(errno));
    }

    vsl_frame_unlock(frame);
    vsl_frame_release(frame);

    return 0;
}

int
main(int argc, char** argv)
{
    int          err;
    VAALContext* vaal;
    void*        zmq;
    void*        pub;
    VAALBox*     boxes     = NULL;
    int          max_boxes = 50;
    float        threshold = 0.25f;
    float        iou       = 0.45f;
    const char*  model     = NULL;
    const char*  engine    = "npu";
    const char*  vslpath   = "/tmp/camera.vsl";
    const char*  puburl    = "ipc:///tmp/detect.pub";

    struct option options[] = {
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'v'},
        {"engine", required_argument, NULL, 'e'},
        {"vsl", required_argument, NULL, 's'},
        {"pub", required_argument, NULL, 'p'},
        {"topic", required_argument, NULL, 't'},
        {"max-boxes", required_argument, NULL, 'm'},
        {"threshold", required_argument, NULL, 'T'},
        {"iou", required_argument, NULL, 'I'},
        {NULL},
    };

    for (;;) {
        int opt = getopt_long(argc, argv, "hve:m:s:p:t:T:I:", options, NULL);
        if (opt == -1) break;

        switch (opt) {
        case 'h':
            printf("detect [hv] [-e engine] [-s path] [-p url] model.rtm\n"
                   "-h, --help\n"
                   "    display help information\n"
                   "-v, --version\n"
                   "    display version information\n"
                   "-m MAX --max-boxes MAX\n"
                   "    maximum detection boxes per frame (default: %d)\n"
                   "-T THRESHOLD, --threshold THRESHOLD\n"
                   "    set the detection threshold (default: %.2f)\n"
                   "-I IOU, --iou IOU\n"
                   "    set the detection iou for nms (default: %.02f)\n"
                   "-e ENGINE, --engine ENGINE\n"
                   "    select the inference engine device [cpu, gpu, npu*]\n"
                   "-s PATH, --vsl PATH\n"
                   "    vsl socket path to capture frames (default: %s)\n"
                   "-p URL, --pub URL\n"
                   "    url for the result message queue (default: %s)\n"
                   "-t TOPIC --topic TOPIC\n"
                   "    publish events under TOPIC (default: %s)\n",
                   max_boxes,
                   threshold,
                   iou,
                   vslpath,
                   puburl,
                   topic);
            return EXIT_SUCCESS;
        case 'v':
            printf("visionpack ipc example - vaal %s vsl %s\n",
                   vaal_version(NULL, NULL, NULL, NULL),
                   vsl_version());
            return EXIT_SUCCESS;
        case 'e':
            engine = optarg;
            break;
        case 'm':
            max_boxes = optarg;
            break;
        case 's':
            vslpath = optarg;
            break;
        case 'p':
            puburl = optarg;
            break;
        case 't':
            topic = optarg;
            break;
        default:
            fprintf(stderr,
                    "invalid parameter %c, try --help for usage\n",
                    opt);
            return EXIT_FAILURE;
        }
    }

    model = argv[optind++];

    /**
     * The VAALContext is used for all VAAL operations and one should be created
     * per-model to be executed by the application.
     */
    vaal = vaal_context_create(engine);
    if (!vaal) {
        fprintf(stderr, "failed to create vaal context\n");
        return EXIT_FAILURE;
    }

    vaal_parameter_setf(vaal, "score_threshold", &threshold, 1);
    vaal_parameter_setf(vaal, "iou_threshold", &iou, 1);

    err = vaal_load_model_file(vaal, model);
    if (err) {
        fprintf(stderr, "failed to load %s: %s\n", model, vaal_strerror(err));
        return EXIT_FAILURE;
    }

    boxes = calloc(max_boxes, sizeof(VAALBox));
    if (!boxes) {
        fprintf(stderr, "allocate boxes: out of memory\n");
        return EXIT_FAILURE;
    }

    err = vaal_parameter_seti(vaal, "max_detection", &max_boxes, 1);
    if (err) {
        fprintf(stderr,
                "failed to set context parameter 'max_detection': %s\n",
                vaal_strerror(err));
        return EXIT_FAILURE;
    }

    printf("loaded model %s\n", model);

    /**
     * The ZeroMQ Context is required for all ZeroMQ API functions.  We create
     * the context then we create our publisher socket which will be used for
     * publishing detection results from VAAL.
     */
    zmq = zmq_ctx_new();
    if (!zmq) {
        fprintf(stderr,
                "failed to create zeromq context: %s\n",
                zmq_strerror(errno));
        return EXIT_FAILURE;
    }

    pub = zmq_socket(zmq, ZMQ_PUB);
    if (!pub) {
        fprintf(stderr,
                "failed to create zeromq socket: %s\n",
                zmq_strerror(errno));
        return EXIT_FAILURE;
    }

    int hwm = 1;
    err     = zmq_setsockopt(pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    if (err) {
        fprintf(stderr,
                "failed to set zeromq option ZMQ_SNDHWM: %s\n",
                zmq_strerror(errno));
        return EXIT_FAILURE;
    }

    int cnf = 1;
    err     = zmq_setsockopt(pub, ZMQ_CONFLATE, &cnf, sizeof(cnf));
    if (err) {
        fprintf(stderr,
                "failed to set zeromq option ZMQ_CONFLATE: %s\n",
                zmq_strerror(errno));
        return EXIT_FAILURE;
    }

    err = zmq_bind(pub, puburl);
    if (err) {
        fprintf(stderr,
                "failed to bind zeromq socket: %s\n",
                zmq_strerror(errno));
        return EXIT_FAILURE;
    }

    printf("publishing results to %s\n", puburl);

    /**
     * The application uses the VideoStream Library for sharing camera frames
     * between the various applications for this demonstration.  We initialize
     * the client to connect to vslpath which should be the end-point into which
     * we inject capture frames using GStreamer and vslsink or a native vslhost
     * application.
     */
    vsl = vsl_client_init(vslpath, NULL, true);
    if (!vsl) {
        fprintf(stderr,
                "failed to connect videostream socket %s: %s\n",
                vslpath,
                strerror(errno));
        return EXIT_FAILURE;
    }

    printf("capturing frames from %s\n", vslpath);

    // 100ms timeout on frame capture.
    vsl_client_set_timeout(vsl, 0.1f);

    /**
     * Install a SIGINT handler so we can cleanup on a control-c keyboard input.
     */
    signal(SIGINT, quit);

    /**
     * There's many different ways for an application to implement its event
     * loop.  Here have a function which reads frames from vsl to send to the
     * model to perform inference then finally publishes results as JSON over
     * the ZeroMQ socket.  The application loop simply runs this function
     * forever.
     */
    while (running) {
        err = handle_vsl(pub, vaal, boxes, max_boxes);
        if (err) { return EXIT_FAILURE; }
    }

    /**
     * Cleanup resources before exiting the application.  This allows us to use
     * something like valgrind to ensure the application has no resource leaks.
     *
     * NOTE: the OpenVX driver required by NPU support will produce a lot of
     * valgrind noise, use the CPU for inference if you wish to test your
     * application for resource leaks.
     */
    zmq_close(pub);
    zmq_ctx_destroy(zmq);

    return EXIT_SUCCESS;
}
