#include <alpha_localization/MarkerLocalization.h>
#include <sstream>
#include <string>
#include <map>

MarkerLocalization::MarkerLocalization(){
  
}
void MarkerLocalization::init(){

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
		 TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 40, 0.001));
  }
}

void MarkerLocalization::loadCameraInfo(){
  std::vector<double> camera_matrix,dist_coeffs,camera_trans;
  private_nh.getParam("/marker_localization/camera_matrix",camera_matrix);
  private_nh.getParam("/marker_localization/dist_coeffs",dist_coeffs);
  private_nh.getParam("/marker_localization/camera_transformation",camera_trans);

  camMatrix = (Mat_<float>(3,3) <<
	       (float)camera_matrix[0],(float)camera_matrix[1],(float)camera_matrix[2],
	       (float)camera_matrix[3],(float)camera_matrix[4],(float)camera_matrix[5],
	       (float)camera_matrix[6],(float)camera_matrix[7],(float)camera_matrix[8]);
  distCoeffs = (Mat_<float>(1,5) << (float)dist_coeffs[0],(float)dist_coeffs[1],(float)dist_coeffs[2],
		(float)dist_coeffs[3],(float)dist_coeffs[4]);
  camera_t = (Mat_<float>(3,1) <<
	      (float)(-camera_trans[1]),(float)(-camera_trans[2]),(float)(camera_trans[0]));
  

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
cv::Mat homography_dlt(const std::vector< cv::Point2f > &x1, const std::vector< cv::Point2f > &x2)
{
  int npoints = (int)x1.size();
  cv::Mat A(2*npoints, 9, CV_32F, cv::Scalar(0));
  // We need here to compute the SVD on a (n*2)*9 matrix (where n is
  // the number of points). if n == 4, the matrix has more columns
  // than rows. The solution is to add an extra line with zeros
  if (npoints == 4)
    A.resize(2*npoints+1, cv::Scalar(0));
  // Since the third line of matrix A is a linear combination of the first and second lines
  // (A is rank 2) we don't need to implement this third line
  for(int i = 0; i < npoints; i++) {      // Update matrix A using eq. 33
    A.at<float>(2*i,3) = -x1[i].x;               // -xi_1
    A.at<float>(2*i,4) = -x1[i].y;               // -yi_1
    A.at<float>(2*i,5) = -1;                     // -1
    A.at<float>(2*i,6) =  x2[i].y * x1[i].x;     //  yi_2 * xi_1
    A.at<float>(2*i,7) =  x2[i].y * x1[i].y;     //  yi_2 * yi_1
    A.at<float>(2*i,8) =  x2[i].y;               //  yi_2
    A.at<float>(2*i+1,0) =  x1[i].x;             //  xi_1
    A.at<float>(2*i+1,1) =  x1[i].y;             //  yi_1
    A.at<float>(2*i+1,2) =  1;                   //  1
    A.at<float>(2*i+1,6) = -x2[i].x * x1[i].x;   // -xi_2 * xi_1
    A.at<float>(2*i+1,7) = -x2[i].x * x1[i].y;   // -xi_2 * yi_1
    A.at<float>(2*i+1,8) = -x2[i].x;             // -xi_2
  }
  // Add an extra line with zero.
  if (npoints == 4) {
    for (int i=0; i < 9; i ++) {
      A.at<float>(2*npoints,i) = 0;
    }
  }
  cv::Mat w, u, vt;
  cv::SVD::compute(A, w, u, vt);
  float smallestSv = w.at<float>(0, 0);
  unsigned int indexSmallestSv = 0 ;
  for (int i = 1; i < w.rows; i++) {
    if ((w.at<float>(i, 0) < smallestSv) ) {
      smallestSv = w.at<float>(i, 0);
      indexSmallestSv = i;
    }
  }
  cv::Mat h = vt.row(indexSmallestSv);
  if (h.at<float>(0, 8) < 0) // tz < 0
    h *=-1;
  cv::Mat _2H1(3, 3, CV_32F);
  for (int i = 0 ; i < 3 ; i++)
    for (int j = 0 ; j < 3 ; j++)
      _2H1.at<float>(i,j) = h.at<float>(0, 3*i+j);
  return _2H1;
}
void pose_from_homography_dlt(const std::vector< cv::Point2f > &xw,
			      const std::vector< cv::Point2f > &xo,
			      cv::Mat &otw, cv::Mat &oRw)
{
  cv::Mat oHw = homography_dlt(xw, xo);
  // Normalization to ensure that ||c1|| = 1
  float norm = sqrt(oHw.at<float>(0,0)*oHw.at<float>(0,0)
		     + oHw.at<float>(1,0)*oHw.at<float>(1,0)
		     + oHw.at<float>(2,0)*oHw.at<float>(2,0));
  oHw /= norm;
  cv::Mat c1  = oHw.col(0);
  cv::Mat c2  = oHw.col(1);
  cv::Mat c3 = c1.cross(c2);
  otw = oHw.col(2);
  for(int i=0; i < 3; i++) {
    oRw.at<float>(i,0) = c1.at<float>(i,0);
    oRw.at<float>(i,1) = c2.at<float>(i,0);
    oRw.at<float>(i,2) = c3.at<float>(i,0);
  }
}


Pose MarkerLocalization::solvePose(std::vector<Point2f> &corners){
  Pose pose;
  
  //  for(int i = 0; i < objectPoints.size(); i++){
  //    std::cout<<objectPoints[i]<<std::endl;
  //  }
  std::vector<Point2f> undistorted_corners;
  undistortPoints(corners,undistorted_corners,camMatrix,distCoeffs);

  std::vector<Point2f> obj;
  std::vector<Point2f> img;
  for(int i = 0; i < 8; i++){
    obj.push_back(Point2f(objectPoints[i].x,objectPoints[i].y));
  }
  Mat R(3,3,CV_32F);
  Mat t(3,1,CV_32F);
  pose_from_homography_dlt(obj,undistorted_corners,t,R);

  //   solvePnPRansac(objectPoints,corners,camMatrix,distCoeffs,rvec,tvec,false,100,8,5,inliers,CV_EPNP);
  //   solvePnP(objectPoints,corners,camMatrix,distCoeffs,rvec,tvec,false,CV_EPNP);
  //  std::vector<Point2f> imagePoints;
  //  projectPoints(objectPoints,rvec,tvec,camMatrix,distCoeffs,imagePoints);
  //  for(int i = 0; i < imagePoints.size(); i++)
  //    std::cout<<imagePoints[i]<<std::endl;
  //  Mat ahrs_rvec = (cv::Mat_<float>(3,1) << (float)(-ahrs_euler.y),0,(float)(ahrs_euler.x));
  Mat rvec;
  //  ahrs_rvec.copyTo(rvec);
  R = R.t();
  cv::Rodrigues(R,rvec);
  //  Mat ahrs_R;
  //  cv::Rodrigues(ahrs_rvec,ahrs_R);
  //  Mat tvec = -camera_t - ahrs_R*t;
  Mat tvec = -camera_t - R*t;


  //  pose.rot = ahrs_euler;
  //  pose.rot.z = 0;
  pose.rot.x = rvec.at<float>(2,0);
  pose.rot.y =-rvec.at<float>(0,0);

  pose.rot.z =-rvec.at<float>(1,0);
  
  pose.pos.x = tvec.at<float>(2,0);
  pose.pos.y =-tvec.at<float>(0,0);
  pose.pos.z =-tvec.at<float>(1,0);
  
  return pose;
}
