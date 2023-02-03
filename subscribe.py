import zmq
import json

topic = "DETECTION"

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect("ipc:///tmp/detect.pub")
sub.subscribe(topic)

while True:
    msg = sub.recv()
    detect = json.loads(msg.removeprefix(topic))
    print(detect)
