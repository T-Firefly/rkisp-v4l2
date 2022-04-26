
## Opencv compilation and installation
* OS： Ubuntu18.04 / Debian 10
* OpenCV version： 3.4.15
* Board: RK3399

1. Build a python3.7 virtual environment
	```
	# Install compilation environments such as gcc and cmake in turn
	# Install python3.7-tk and python3.7-dev
	# Install virtualenv virtual environment
	sudo apt install gcc cmake git build-essential \
	python3-tk python3.7-dev \
	virtualenv
	```

1. Create a python3.7 virtual environment
	```
	virtualenv -p /usr/bin/python3.7m /home/firefly/venv
    # Use a virtual environment, if you want to quit, you can enter deactivate in the terminal
	source /home/firefly/venv/bin/activate
	```

1. Install the required environment packages for Opencv
	```
	# Install the compilation environment, gtk package and related codec libraries
	sudo apt install cmake build-essential libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

    # Install Python-numpy
    apt install python-numpy
	```

1. Download Opencv
    ```
    # Create folder
	mkdir opencv
	cd opencv

	# Download opencv-3.4.15.zip
	wget https://github.com/opencv/opencv/archive/refs/tags/3.4.15.zip
	unzip opencv-3.4.15.zip
    ```

1. Configure
    ```
    mkdir build
    cd build

    # Making connections between OpenCV and Python3
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D 	PYTHON_DEFAULT_EXECUTABLE=$(python -c "import sys; print(sys.executable)")\
    -D PYTHON3_EXECUTABLE=$(python -c "import sys; print(sys.executable)")  \
    -D PYTHON3_NUMPY_INCLUDE_DIRS=$(python -c "import numpy; print (numpy.get_include())")  \
    -D PYTHON3_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")  \
    ../opencv-3.4.15
    ```

1. To support Gstreamer API, perform the following configuration operations, otherwise skip
    ```
        sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
        cmake -D WITH_GSTREAMER=ON ../opencv-3.4.15
    ```
1. Compile and install
    ```
        # If the board memory does not exceed 2G, it is recommended not to exceed -j4
        make -j6
        # The installation process requires sudo privileges
        sudo make install
    ```

## Test
1. Test
    ```
        source /home/firefly/venv/bin/activate
        python3 opencv_gst_test.py
    ```

2. opencv_gst_test.py code
    ```
    import numpy as np
    import cv2 as cv
    import os
    import time

    cap = cv.VideoCapture('v4l2src device=/dev/video1 ! video/x-raw, format=NV12, width=640, height=480, framerate=30/1 ! videoconvert ! appsink', cv.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Cannot capture from camera. Exiting.")
        os._exit()
    last_time = time.time()

    while(True):

        ret, frame = cap.read()
        this_time = time.time()
        print (str((this_time-last_time)*1000)+'ms')
        last_time = this_time;
        cv.imshow('frame', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
    ```