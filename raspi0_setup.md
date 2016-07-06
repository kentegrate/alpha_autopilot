# Setup needed for Raspberry Pi Zero

### OS Installation

Download Raspbian Jessie Lite from [here](https://www.raspberrypi.org/downloads/raspbian/), and install the image into a micro SD card (minimum 8GB required)

### OpenCV 3.1.0 Build and Installation (About ?? hours)

First, follow instructions on [this page](http://www.pyimagesearch.com/2015/12/14/installing-opencv-on-your-raspberry-pi-zero/) to Step #3. 
Then, install git and grab the opencv and opencv_contrib repository.
Go to the opencv respository and make a directory named 'build' and go in there, and type cmake -DOPENCV_EXTRA_MODULES=<opencv_contrib/modules path> ../
