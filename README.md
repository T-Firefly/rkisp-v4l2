# This code is based on the v4l2 camera data capture and display code of the Rockchip chip platform

## Software requirements:
* Opencv
* Cmake

## Example:
```
apt-get install -y git libopencv-dev cmake libdrm-dev g++ librga-dev
cmake ./
make

sudo -u firefly DISPLAY=:0 ./rkisp_demo -c 300 -d /dev/video0 -w 640 -h 480
or
sudo -u firefly DISPLAY=:0 ./rkisp_demo -c 300 -d /dev/video5 -w 640 -h 480
```

# The following program demonstrates how to call MIPI camera in C and C++, and how to use OpenCV for image display.

## Example:
```
cd v4l2_simple_demo/
make
# open /dev/video0
sudo -u firefly DISPLAY=:0 ./opencv
```
