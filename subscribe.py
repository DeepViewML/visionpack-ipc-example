import zmq
import json

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("ipc:///tmp/detect.pub")
sub.subscribe("DETECTION")

while True:
    msgs = sub.recv_multipart()
    detect = json.loads(msgs[1])
    print(detect)
