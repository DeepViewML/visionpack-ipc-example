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

#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <deepview_rt.h>
#include <vaal.h>
#include <videostream.h>

#include "json.hpp"
#include "zmq.hpp"

using nlohmann::json;

namespace data
{
struct box {
    float xmin;
    float xmax;
    float ymin;
    float ymax;
};

struct object {
    box         bbox;
    float       score;
    std::string label;
};

struct result {
    int64_t             timestamp;
    int64_t             boxes_ns;
    int64_t             load_ns;
    int64_t             model_ns;
    int                 fps;
    std::vector<object> objects;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(box, xmin, xmax, ymin, ymax)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(object, bbox, score, label)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(result,
                                                timestamp,
                                                boxes_ns,
                                                load_ns,
                                                model_ns,
                                                fps,
                                                objects)

} // namespace data

int
main(int argc, char** argv)
{
    int         err;
    const char* topic   = "DETECTION";
    const char* vslpath = "/tmp/camera.vsl";

    int display_width  = 1280;
    int display_height = 720;

    cv::namedWindow("overlay", cv::WINDOW_NORMAL);
    cv::setWindowProperty("overlay",
                          cv::WindowPropertyFlags::WND_PROP_FULLSCREEN,
                          1);

    /**
     * This application will receive detection events using a ZeroMQ pub/sub
     * socket where the detection application publishes the results and this
     * overlay application subscribes to the results.
     */
    zmq::context_t ctx;
    zmq::socket_t  sub(ctx, zmq::socket_type::sub);
    sub.set(zmq::sockopt::conflate, 1);
    sub.set(zmq::sockopt::rcvhwm, 1);
    sub.set(zmq::sockopt::subscribe, topic);
    sub.connect("ipc:///tmp/detect.pub");

    /**
     * We will receive the camera frames using VideoStream Library then map and
     * convert them to an OpenCV matrix onto which we will draw the bounding
     * boxes and display the results on-screen.
     */
    auto vsl = vsl_client_init(vslpath, NULL, false);
    if (!vsl) {
        fprintf(stderr,
                "failed to connect videostream socket %s: %s\n",
                vslpath,
                strerror(errno));
        return EXIT_FAILURE;
    }

    /**
     * We will be using VAAL to optimize the conversion of the VSL Frame to the
     * size and pixel format required for OpenCV.  This uses the same method of
     * optimized frame loading into a model which VAAL uses but can also be done
     * without a model by providing a target NNTensor for the operation.
     */
    auto vaal = vaal_context_create("cpu");
    if (!vaal) {
        fprintf(stderr, "failed to create vaal context\n");
        return EXIT_FAILURE;
    }

    auto          image         = nn_tensor_init(nullptr, nullptr);
    const int32_t image_shape[] = {display_height, display_width, 3};
    err = nn_tensor_alloc(image, NNTensorType_U8, 3, image_shape);
    if (err) {
        fprintf(stderr,
                "failed to allocate image tensor: %s\n",
                nn_strerror(NNError(err)));
        return EXIT_FAILURE;
    }

    for (;;) {
        /**
         * To keep the sample simple we sequentially request the next frame and
         * then the results, which will likely be out of sync but near enough
         * for the visual overlays to be correct.
         */
        auto frame = vsl_frame_wait(vsl, 0);
        if (!frame) {
            fprintf(stderr, "failed to acquire frame: %s\n", strerror(errno));
            return EXIT_FAILURE;
        }

        /**
         * Loads the VSL frame into our image NNTensor which will trigger the
         * pixel format conversion from YUV to RGB and resize operation to the
         * image tensor's size.
         */
        err = vaal_load_frame_dmabuf(vaal,
                                     image,
                                     vsl_frame_handle(frame),
                                     vsl_frame_fourcc(frame),
                                     vsl_frame_width(frame),
                                     vsl_frame_height(frame),
                                     nullptr,
                                     0);
        vsl_frame_release(frame);

        if (err) {
            fprintf(stderr,
                    "failed to load frame: %s\n",
                    vaal_strerror(VAALError(err)));
            return EXIT_FAILURE;
        }

        /**
         * Maps the tensor to get the raw pixel data which will be used as the
         * data for our cvimage.  Then we create our display mat and perform
         * the RGB2BGR conversion required for OpenCV.
         *
         * Note: while the conversion here could potentially be skipped you
         * would still need a memcpy into the display mat as the image tensor
         * is not stored in cache optimized memory leading to slow draw times
         * by OpenCV.  If using hardware accelerated graphics such as OpenGL
         * there are better ways to handle these large image buffers without
         * costly copy or conversion operations.
         */
        auto pix = static_cast<uint8_t*>(nn_tensor_maprw(image));
        if (!pix) {
            fprintf(stderr, "failed to map image: no data\n");
            continue;
        }

        cv::Mat cvimage(display_height, display_width, CV_8UC3, pix);
        cv::Mat display(display_height, display_width, CV_8UC3);
        cv::cvtColor(cvimage, display, cv::COLOR_RGB2BGR);
        cvimage.release();
        nn_tensor_unmap(image);

        /**
         * Read the current message from the detection application.  Depending
         * on processing time to get our cv::mat this is likely to be out of
         * sync with our image but within a couple frames and visually correct
         * as long as the objects are not moving too fast.
         */
        zmq::message_t msg;
        auto           ret = sub.recv(msg);
        if (!ret) {
            fprintf(stderr, "received empty message\n");
            continue;
        }

        size_t      topic_len = strlen(topic);
        const char* data      = (const char*) msg.data();
        size_t      size      = msg.size();

        if (size < topic_len) { continue; }
        data += topic_len;
        size -= topic_len;

        auto j   = json::parse(data, data + size);
        auto res = j.get<data::result>();

        /**
         * Draws the bounding box rectangles over the image then draw window.
         */
        for (const auto& obj : res.objects) {
            const auto& box = obj.bbox;
            cv::Rect    rect{int(box.xmin * display_width),
                          int(box.ymin * display_height),
                          int((box.xmax - box.xmin) * display_width),
                          int((box.ymax - box.ymin) * display_height)};
            cv::rectangle(display, rect, cv::Scalar(0, 255, 0));
        }

        cv::imshow("overlay", display);
        cv::waitKey(1);
    }

    return EXIT_SUCCESS;
}