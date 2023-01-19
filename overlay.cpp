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

#include <videostream.h>

#include "json.hpp"
#include "zmq.hpp"

using nlohmann::json;

int main(int argc, char **argv) {
    zmq::context_t ctx;
    zmq::socket_t  sub(ctx, zmq::socket_type::sub);

    sub.set(zmq::sockopt::subscribe, "");
    sub.connect("ipc:///tmp/detect.pub");

    for (;;) {
        zmq::message_t msg;

        auto res = sub.recv(msg, zmq::recv_flags::none);
        if (!res) { fprintf(stderr, "received empty message\n");
            continue;
        }

        auto detect = json::parse(msg.to_string());
        printf("DETECTIONS: %s\n", detect.dump(4).c_str());
    }

    return EXIT_SUCCESS;
    }