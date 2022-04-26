# Directory Structure
```
.
├── README.md
├── mipi_video_demo
│   ├── OpenCV_Python
│   └── v4l2_simple_demo
└── rkisp_demo
```

# 1. rkisp_demo
This code is based on the v4l2 camera data capture and display code of the Rockchip chip platform.

```
# E.g.
cd rkisp_demo
apt-get install -y git libopencv-dev cmake libdrm-dev g++ librga-dev
cmake ./
make

sudo -u firefly DISPLAY=:0 ./rkisp_demo -c 300 -d /dev/video0 -w 640 -h 480
or
sudo -u firefly DISPLAY=:0 ./rkisp_demo -c 300 -d /dev/video5 -w 640 -h 480
```

# 2. v4l2_simple_demo
The following program demonstrates how to call MIPI camera in C and C++, and how to use OpenCV for image display.

```
# E.g.
cd mipi_video_demo/v4l2_simple_demo/
make
# open /dev/video0
sudo -u firefly DISPLAY=:0 ./opencv
```

# 3. OpenCV_Python
The following program demonstrates how to call MIPI camera in Python, and how to use OpenCV for image display.But need to compile OpenCV and add gstreamer support.

```
# E.g.
cd mipi_video_demo/OpenCV_Python
Python3 opencv_gst_test.py
```
