CC ?= gcc
CFLAGS ?=
CXX ?= g++
CXXFLAGS ?=
OPENCV ?= $(shell pkg-config --cflags opencv4)

all: detect overlay

detect: detect.c
	$(CC) $(CFLAGS) -odetect detect.c flex.c writer.c -lzmq -lvaal -lvideostream

overlay: overlay.cpp
	$(CXX) $(CXXFLAGS) $(OPENCV) -ooverlay overlay.cpp -lzmq -lvideostream -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs

clean:
	$(RM) detect overlay
