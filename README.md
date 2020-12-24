# This code is based on the v4l2 camera data capture and display code of the Rockchip chip platform

## Software requirements:
* Opencv
* Cmake

## Example:
```
apt-get install -y libopencv-dev
cmake ./
make

sudo -u firefly DISPLAY=:0 ./rkisp_demo -c 300 -d /dev/video0 -w 640 -h 480
or
sudo -u firefly DISPLAY=:0 ./rkisp_demo -c 300 -d /dev/video5 -w 640 -h 480
```
