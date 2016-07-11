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
  Mat thresh_img,thresh_img2;
  //  adaptiveThreshold(gray,thresh_img, 255, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,9,10);
  //  imshow("image1",thresh_img);
  threshold(gray, thresh_img, 100, 255, THRESH_BINARY);
  imshow("image2",thresh_img);
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
    if(area_rect > area_raw*1.5)
      hierarchy[i][2] = -1;
  }
  vector<int> inlier_idx;
  for(int i = 0; i < contours.size(); i++){
    if(hierarchy[i][2] == -1)
      continue;
    
    int j = hierarchy[i][2];
    while(j!=-1){
      if(area[j] < area[i] *0.9 && area[j] > area[i]*0.1 && area[j] > 25 ){
	Moments m_i = moments(contours[i]);
	Moments m_j = moments(contours[j]);
	if(pow(m_i.m10/m_i.m00 - m_j.m10/m_j.m00, 2)+
	   pow(m_i.m01/m_i.m00 - m_j.m01/m_j.m00, 2) < 100)
	  inlier_idx.push_back(i);
	//	inlier_idx.push_back(j);
	//	break;
      }
      j = hierarchy[j][1];
    }
  }
  double max_area = 0;
  double max_idx = -1;
  for(int i = 0; i < inlier_idx.size(); i++){
    if(area_2[inlier_idx[i]] > max_area){
      max_area = area_2[inlier_idx[i]];
      max_idx = inlier_idx[i];
    }
  }
  if(max_idx!=-1){
    filtered.push_back(contours[max_idx]);
    filtered.push_back(contours[hierarchy[max_idx][2]]);
  }
  contours = filtered;
  vector<Point2f> corners;
  for(int i = 0; i < contours.size(); i++){
    RotatedRect r = minAreaRect(contours[i]);
    Point2f pts[4];
    r.points(pts);
    for(int j = 0; j < 4; j++){
      corners.push_back(pts[j]);
    }
  }
  if(corners.size() > 0){
    cornerSubPix(gray, corners, Size(2,2), 
		 Size(-1, -1), 
		 TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.1));
    for(int i = 0; i < corners.size(); i++){
      std::cout<<corners[i]<<std::endl;
      circle(output, corners[i], 5, Scalar(0,200,100), 3, 4);
    }

  vector<Point3f> objectPoints;
  objectPoints.push_back(Point3f(-1.85,7.9,0));
  objectPoints.push_back(Point3f(-1.85,-2.3,0));
  objectPoints.push_back(Point3f(7.3,-2.3,0));  
  objectPoints.push_back(Point3f(7.3,7.9,0));

  objectPoints.push_back(Point3f(0,5.5,0));
  objectPoints.push_back(Point3f(0,0,0));
  objectPoints.push_back(Point3f(5.4,0,0));  
  objectPoints.push_back(Point3f(5.4,5.5,0));
  Mat distCoeffs = (Mat_<float>(1,5) << 0.182276,-0.533582,0.000520,-0.001682,0.000000);
  Mat camMatrix  = (Mat_<float>(3,3) << 
		    743.023418,0.000000,329.117496,
		    0.000000,745.126083,235.748102,
		    0.000000,0.000000,1.000000);

  Mat rvec,tvec;
  solvePnP(objectPoints, corners, camMatrix, distCoeffs, rvec, tvec);
 //  std::cout<<rvec<<std::endl;
  std::cout<<tvec<<std::endl;
  }
  //  std::cout<<"computation time "<<get_dtime()-start<<std::endl;
}


int main(int argc, char *argv[]){
  ros::init(argc, argv, "chessboard_test");
  ros::NodeHandle nh;
  namedWindow("detection_result");
  startWindowThread();
  namedWindow("image1");
  namedWindow("image2");
  namedWindow("input");
  startWindowThread();

  VideoCapture input;
  input.open(0);
  Mat image,result;
  while(input.grab() && ros::ok()){
    input.retrieve(image);
    process(image,result);
    imshow("detection_result",result);
    imshow("input",image);
  }
  return 0;
}
