#include <alpha_localization/MarkerLocalization.h>
#include <sstream>
#include <string>
#include <map>

MarkerLocalization::MarkerLocalization(){
  
}
void MarkerLocalization::init(){
  ahrs_sub = private_nh.subscribe("/pose",10,&MarkerLocalization::ahrsCB,this);
}
void MarkerLocalization::ahrsCB(const alpha_msgs::FilteredState::ConstPtr msg){
  ahrs_euler.x = msg->roll;
  ahrs_euler.y = msg->pitch;
  ahrs_euler.z = msg->yaw;
}
void MarkerLocalization::loadMarkerPosition(){
  std::vector<std::vector<double> > marker_corners;
  for(int i = 0; i < 8; i++){
    std::stringstream ss;
    ss<<i;

    std::string point_name = "/marker_localization/point_"+ss.str();

    std::vector<double> corner;
    private_nh.getParam(point_name,corner);
    marker_corners.push_back(corner);
  }
  //   std::cout<<marker_corners[0].size()<<std::endl;
   for(int i = 0; i < 8; i++){
    Point3f point((float)(-marker_corners[i][1]),
		  (float)(-marker_corners[i][2]),
		  (float)(marker_corners[i][0]));
    objectPoints.push_back(point);
    }

}
void MarkerLocalization::getCVCorners(Mat &input, std::vector<Point2f> &corners, Mat &debug_image){



  //  cvtColor(input,gray,CV_BGR2GRAY);
  Mat thresh_img,thresh_img2;
  
  threshold(input, thresh_img, 100, 255, THRESH_BINARY);

  thresh_img.copyTo(debug_image);
  //  imshow("detection_result",thresh_img);
  //  output = Mat::zeros(thresh_img.rows, thresh_img.cols, CV_8UC3);
  std::vector<std::vector<Point> > contours,filtered;
  std::vector<Vec4i> hierarchy;

  findContours( thresh_img, contours, hierarchy,
		CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

  std::vector<float> area;
  std::vector<float> area_2;
  std::vector<int> index,index_2;
  for(int i = 0; i < contours.size(); i++){
    RotatedRect r = minAreaRect(contours[i]);
    float area_rect = r.size.height * r.size.width;
    float area_raw = contourArea(contours[i]);
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
      if(area[j] < area[i] *0.2 && area[j] > area[i]*0.05 && area[j] > 25 ){
	//	std::cout<<"i/j "<<area[i]/area[j]<<std::endl;
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
  float max_area = 0;
  float max_idx = -1;
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

  for(int i = 0; i < contours.size(); i++){
    RotatedRect r = minAreaRect(contours[i]);
    Point2f pts[4];
    r.points(pts);
    for(int j = 0; j < 4; j++){
      corners.push_back(pts[j]);
    }
  }
  if(corners.size() > 0){
    cornerSubPix(input, corners, Size(5,5), 
		 Size(-1, -1), 
		 TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.1));
  }
}

void MarkerLocalization::loadCameraInfo(){
  std::vector<double> camera_matrix,dist_coeffs;
  private_nh.getParam("/marker_localization/camera_matrix",camera_matrix);
  private_nh.getParam("/marker_localization/dist_coeffs",dist_coeffs);

  camMatrix = (Mat_<float>(3,3) <<
	       (float)camera_matrix[0],(float)camera_matrix[1],(float)camera_matrix[2],
	       (float)camera_matrix[3],(float)camera_matrix[4],(float)camera_matrix[5],
	       (float)camera_matrix[6],(float)camera_matrix[7],(float)camera_matrix[8]);
  distCoeffs = (Mat_<float>(1,5) << (float)dist_coeffs[0],(float)dist_coeffs[1],(float)dist_coeffs[2],
		(float)dist_coeffs[3],(float)dist_coeffs[4]);
  

}

void MarkerLocalization::sortCorners(std::vector<Point2f> &corners){
  if(corners.size()!=8)
    return;
  std::vector<std::pair<float,float> > x_sorted_points;
  for(int i = 0; i < corners.size(); i++)
    x_sorted_points.push_back(std::make_pair(corners[i].x,corners[i].y));

  std::sort(x_sorted_points.begin(),x_sorted_points.end());

  std::vector<Point2f> sorted_corners;
  
  sort_helper(sorted_corners,x_sorted_points[0],x_sorted_points[1],true);
  sort_helper(sorted_corners,x_sorted_points[6],x_sorted_points[7],false);
  sort_helper(sorted_corners,x_sorted_points[2],x_sorted_points[3],true);
  sort_helper(sorted_corners,x_sorted_points[4],x_sorted_points[5],false);  
  corners = sorted_corners;

}
void MarkerLocalization::sort_helper(std::vector<Point2f> &sorting, std::pair<float,float> pt1, std::pair<float,float> pt2, bool reverse){
  std::vector<std::pair<float,float> > adding_points;
  adding_points.push_back(pt1);
  adding_points.push_back(pt2);

  float y1 = pt1.second;
  float y2 = pt2.second;    

  if(((y1 > y2 )&&(!reverse)) || ((y1 < y2)&&(reverse)))
    std::swap(adding_points[0],adding_points[1]);

  for(int i = 0; i < 2; i++)
    sorting.push_back(Point2f(adding_points[i].first,adding_points[i].second));

}

Pose MarkerLocalization::solvePose(std::vector<Point2f> &corners){
  Pose pose;
  
  Mat R,R_t;
  Mat rvec = Mat::zeros(3,1,CV_64FC1);
  Mat tvec = Mat::zeros(3,1,CV_64FC1);
  
  //  for(int i = 0; i < objectPoints.size(); i++){
  //    std::cout<<objectPoints[i]<<std::endl;
  //  }
  Mat inliers;
  //   solvePnPRansac(objectPoints,corners,camMatrix,distCoeffs,rvec,tvec,false,100,8,5,inliers,CV_EPNP);
   solvePnP(objectPoints,corners,camMatrix,distCoeffs,rvec,tvec,false,CV_EPNP);
  std::vector<Point2f> imagePoints;
  //  projectPoints(objectPoints,rvec,tvec,camMatrix,distCoeffs,imagePoints);
  //  for(int i = 0; i < imagePoints.size(); i++)
  //    std::cout<<imagePoints[i]<<std::endl;
  Mat ahrs_rvec = (cv::Mat_<double>(3,1) << (double)(-ahrs_euler.y),0,(double)(ahrs_euler.x));
  Mat ahrs_R;
  cv::Rodrigues(ahrs_rvec,ahrs_R);
		   
  tvec = -ahrs_R*tvec;


  pose.rot = ahrs_euler;
  pose.rot.z = 0;
  pose.pos.x = tvec.at<double>(2,0);
  pose.pos.y =-tvec.at<double>(0,0);
  pose.pos.z =-tvec.at<double>(1,0);
  
  return pose;
}
