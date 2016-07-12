# Setup needed for Raspberry Pi Zero

### OS Installation

Download Raspbian Jessie Lite from [here](https://www.raspberrypi.org/downloads/raspbian/), and install the image into a micro SD card (minimum 8GB required)

sudo apt-get update && sudo apt-get upgrade

### OpenCV 2.4.8 installation

sudo apt-get install libopencv-dev

### ROS Installation

### Raspi-Cam Driver installation
http://seesaawiki.jp/yattemiyo/d/Raspberry%20Pi3%A4%C7%A5%AB%A5%E1%A5%E9%A5%E2%A5%B8%A5%E5%A1%BC%A5%EB%A4%F2OpenCV%A4%C7%BB%C8%A4%A6

sudo apt-get install libv4l-dev
insert a new line "bcm2835-v4l2" at the end of /etc/modules