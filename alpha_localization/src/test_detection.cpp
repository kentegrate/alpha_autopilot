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
void process(Mat &input, Mat &output){
  double start = get_dtime();
  Mat gray;
  cvtColor(input,gray,CV_BGR2GRAY);
  //  adaptiveThreshold(gray, output, 255, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,9,12);
  Mat thresh_img;
  threshold(gray, thresh_img, 100, 255, THRESH_BINARY);
  //  std::vector<std::vector<Point2f> > contours;
  output = Mat::zeros(thresh_img.rows, thresh_img.cols, CV_8UC3);
  std::vector<std::vector<Point> > contours,filtered;
  std::vector<Vec4i> hierarchy;
  //thresh_img.copyTo(output);
  findContours( thresh_img, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

  // トップレベルにあるすべての輪郭を横断し，
  // 各連結成分をランダムな色で描きます．
  int idx = 0;
  /*  for(int i = 0; i < hierarchy.size(); i++){
    std::cout<<hierarchy[i]<<std::endl;
    }*/
  std::vector<double> area;
  std::vector<double> area_2;
  std::vector<int> index,index_2;
  for(int i = 0; i < contours.size(); i++){
    RotatedRect r = minAreaRect(contours[i]);
    double area_rect = r.size.height * r.size.width;
    double area_raw = contourArea(contours[i]);
    area.push_back(area_rect);
    area_2.push_back(area_raw);
    if(area_rect > area_raw*1.3)
      hierarchy[i][2] = -1;
  }

  for(int i = 0; i < contours.size(); i++){
    if(hierarchy[i][2] == -1)
      continue;
    
    int j = hierarchy[i][2];
    while(j!=-1){
      if(area[j] < area[i] *0.6 && area[j] > area[i]*0.4){
	filtered.push_back(contours[i]);
	filtered.push_back(contours[j]);
	//	break;
      }
      j = hierarchy[j][1];
    }

  }

  contours = filtered;
  
  /*  for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
      Scalar color( rand()&255, rand()&255, rand()&255 );
      drawContours( output, contours, idx, color, CV_FILLED, 8, hierarchy );
      }*/
  //  findContours(thresh_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
  //  thresh_img.copyTo(output);
  
  drawContours(output, contours, -1, Scalar(50,50,50));

  std::cout<<"computation time "<<get_dtime()-start<<std::endl;
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
