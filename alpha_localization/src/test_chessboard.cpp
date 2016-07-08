#include <ros/ros.h>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace cv;

double get_dtime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  //ミリ秒を計算
  return ((double)(tv.tv_sec)*1000 + (double)(tv.tv_usec)*0.001); //★
}

float calc_dist(Point2f &p1, Point2f &p2){
  return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));

}
void process(Mat &input_high, Mat &output){
  double start = get_dtime();
  Size pattern_size(3,3);

  Mat corners,input;
  resize(input_high, input, Size(), 0.5, 0.5);
  input.copyTo(output);
  if(!findChessboardCorners(input, pattern_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH+  CV_CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK))
    return;

  Mat gray(input.rows,input.cols,CV_8UC1);
  cvtColor(input,gray,CV_BGR2GRAY);
  Mat reshaped_corners = corners.reshape(3,3);
  float min_dist = 10000000;
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 2; j++){
      min_dist = std::min(min_dist, calc_dist(reshaped_corners.at<Point2f>(i,j),
					      reshaped_corners.at<Point2f>(i,j+1)));
      min_dist = std::min(min_dist, calc_dist(reshaped_corners.at<Point2f>(j,i),
					      reshaped_corners.at<Point2f>(j+1,i)));
    }
  }
      
  int radius = std::ceil(min_dist*0.5);
  Size chess_size(radius,radius);      
  //  cornerSubPix(gray,corners, chess_size, Size(-1, -1), TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.1));
  std::cout<<"computation time "<<get_dtime()-start<<std::endl;
  drawChessboardCorners(output,pattern_size,corners, true);

}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "chessboard_test");
  ros::NodeHandle nh;
  namedWindow("detection_result");
  startWindowThread();

  VideoCapture input;
  input.open(0);
  Mat image,result;
  while(input.grab() && ros::ok()){
    input.retrieve(image);
    process(image,result);
    imshow("detection_result",result);
  }
  return 0;
}
