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

extern "C" {
#include <videostream.h>
}

#include "json.hpp"
#include "zmq.hpp"
#include "zmq_addon.hpp"

using nlohmann::json;

int
main(int argc, char** argv)
{
    const char* vslpath = "/tmp/camera.vsl";

    // cv::namedWindow("overlay", cv::WINDOW_NORMAL);

    /**
     * This application will receive detection events using a ZeroMQ pub/sub
     * socket where the detection application publishes the results and this
     * overlay application subscribes to the results.
     */
    zmq::context_t ctx;
    zmq::socket_t  sub(ctx, zmq::socket_type::sub);
    sub.set(zmq::sockopt::subscribe, "DETECTION");
    sub.connect("ipc:///tmp/detect.pub");

    /**
     * We will receive the camera frames using VideoStream Library then map and
     * convert them to an OpenCV matrix onto which we will draw the bounding
     * boxes and display the results on-screen.
     *
     * NOTE: Currently the demo is setup to receive the YUYV frames and handle
     * conversion using OpenCV.  This is not well-optimized, a future version
     * of VSL will provide optimized image conversion functions.
     */
    auto vsl = vsl_client_init(vslpath, NULL, false);
    if (!vsl) {
        fprintf(stderr,
                "failed to connect videostream socket %s: %s\n",
                vslpath,
                strerror(errno));
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

        auto err = vsl_frame_trylock(frame);
        if (err) {
            fprintf(stderr, "failed to lock frame: %s\n", strerror(errno));
            vsl_frame_release(frame);
            return EXIT_FAILURE;
        }

        auto width  = vsl_frame_width(frame);
        auto height = vsl_frame_height(frame);
        auto fourcc = vsl_frame_fourcc(frame);

        printf("frame %dx%d %c%c%c%c\n",
               width,
               height,
               fourcc,
               fourcc >> 8,
               fourcc >> 16,
               fourcc >> 24);

        auto    pix = vsl_frame_mmap(frame, NULL);
        cv::Mat yuv(height, width, CV_8UC2, pix);
        cv::Mat rgb(height, width, CV_8UC3);
        cv::cvtColor(yuv, rgb, cv::COLOR_YUV2BGR_YUY2);

        vsl_frame_unlock(frame);
        vsl_frame_release(frame);

        /**
         * Read the current message from the detection application.  Depending
         * on processing time to get our cv::mat this is likely to be out of
         * sync with our image but within a couple frames and visually correct
         * as long as the objects are not moving too fast.
         */
        std::vector<zmq::message_t> msgs;
        auto ret = zmq::recv_multipart(sub, std::back_inserter(msgs));
        if (!ret) {
            fprintf(stderr, "received empty message\n");
            continue;
        }

        auto detect = json::parse(msgs[1].to_string());
        printf("%s: %s\n", msgs[0].to_string().c_str(), detect.dump(4).c_str());

        // cv::imshow("overlay", rgb);
        // cv::imwrite("/tmp/overlay.jpg", rgb);
    }

    return EXIT_SUCCESS;
}