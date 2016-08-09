#ifndef ALPHA_DRIVERS_RASPI_CAM_H
#define ALPHA_DRIVERS_RASPI_CAM_H
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <unistd.h>
#include "RaspiCamCV.h"

class RaspiCam{
 private:
  RaspiCamCvCapture * capture;
 public:
  void init(int width=640,int height=480,int framerate=15,
       bool monochrome = false);
  ~RaspiCam(){
    releaseCamera();
  }
  void getFrame(cv::Mat &frame);
  void releaseCamera();

};



#endif //ALPHA_DRIVERS_RASPI_CAM_H
