CC ?= gcc
CFLAGS ?=
CXX ?= g++
CXXFLAGS ?=

all: detect overlay

detect: detect.c
	$(CC) $(CFLAGS) -odetect detect.c flex.c writer.c -lzmq -lvaal -lvideostream

overlay: overlay.cpp
	$(CXX) $(CXXFLAGS) -ooverlay overlay.cpp -lzmq -lvideostream

clean:
	$(RM) detect overlay
