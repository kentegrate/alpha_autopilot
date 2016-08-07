#include <cv.h>
#include <stdio.h>
#include <unistd.h>
#include <alpha_drivers/RaspiCam.h>


void RaspiCam::init(int width,int height,int framerate,
		   bool monochrome){
  RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
  config->width = width;
  config->height = height;
  config->bitrate=0;
  config->framerate=framerate;
  config->monochrome = monochrome;
  capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
  free(config);
}

void RaspiCam::getFrame(cv::Mat &frame){
  cv::Mat new_frame(raspiCamCvQueryFrame(capture));

  new_frame.copyTo(frame);
}
void RaspiCam::releaseCamera(){
  raspiCamCvReleaseCapture(&capture);
}


