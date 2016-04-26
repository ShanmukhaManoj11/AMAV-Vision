#include "ros/ros.h"
#include "vision/CamMsg.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/OpticalFlowRad.h"

float height=0;
/*void xyz_read(const geometry_msgs::PoseStamped::ConstPtr& xyzmsg)
{
      height = xyzmsg->pose.position.z;
      //ROS_INFO_STREAM(flow_read);
      //ROS_INFO("z: [%f]", z_in);
}*/

void px4terminal(const mavros_msgs::OpticalFlowRad& msg)
{
	float sonar = msg.distance;
	height = sonar;

}

//-----------------------------------------------------------------------------------------------------------------------------------------
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//-----------------------------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------------------------
using namespace cv;
using namespace std;

typedef struct msg {
  float range;
  float heading_angle;
} cam_msg;

string WindowName="cam";

cam_msg Process(Mat img){

   int iLowH1=159, iHighH1=179; //Hue range for RED color
  int iLowH2=0, iHighH2=20;
  int iLowS=100, iHighS=255; //Saturation range for RED color
  int iLowV=150, iHighV=255; //Value range for RED color

    cam_msg m;
    m.range=NAN; m.heading_angle=NAN;
    medianBlur(img,img,3);
    Mat imgHSV;
    cvtColor(img,imgHSV,COLOR_BGR2HSV); //Convert the captured image from BGR to HSV
    //blur(imgHSV,imgHSV,Size(3,3));
    Mat imgThresholded1,imgThresholded2,imgThresholded;
    inRange(imgHSV,Scalar(iLowH1,iLowS,iLowV),Scalar(iHighH1,iHighS,iHighV),imgThresholded1); //Threshold the image
    inRange(imgHSV,Scalar(iLowH2,iLowS,iLowV),Scalar(iHighH2,iHighS,iHighV),imgThresholded2);
    imgThresholded=imgThresholded1+imgThresholded2;
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    //morphological closing (fill small holes in the foreground)
    dilate(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5))); 
    erode(imgThresholded,imgThresholded,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    Mat copy=imgThresholded.clone();

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(copy,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //find moments
    vector<Moments> mu;
    // Get the mass centers
    vector<Point2f> mc;
    for(int i=0;i<contours.size();i++){
      if(contourArea(contours[i])>2000){
        drawContours(img,contours,i,Scalar(25,145,65),3);
        mu.push_back(moments(contours[i], false));
        Moments p=mu.back();
        mc.push_back(Point2f(p.m10/p.m00 , p.m01/p.m00));
        Point2f c=mc.back(); 
        //cout<<"X: "<<c.x<<" Y: "<<c.y<<endl;
        circle(img, c, 3, Scalar(255, 0, 0));
      }
    }
    Point2f ic=Point2f(img.cols/2.0f,img.rows/2.0f);
    circle(img, ic, 5, Scalar(0, 0, 255));

    //caliberation
    float fx=1054;
    float fy=1124;

  if(mc.size()>0){
   float x_img=0, y_img=0;
   for(int i=0;i<mc.size();i++){
     x_img=x_img+mc[i].x;
     y_img=y_img+mc[i].y;
   }
   x_img=x_img/mc.size();
   y_img=y_img/mc.size();
    float dist_in_img=abs(y_img-ic.y);
    //cout<<dist_in_img<<endl;
    float theta=atan2(dist_in_img,fy);
    //cout<<theta<<endl;
    float alpha;
    if(y_img<ic.y)
      alpha=(M_PI*5.2/18)+theta;
    else
      alpha=(M_PI*5.2/18)-theta;
    float height_act=height;
    float X_act=tan(alpha)*height_act;
    //cout<<"X: "<<X_act<<endl;

    dist_in_img=abs(x_img-ic.x);
    float l=height_act/cos(M_PI*5.2/18);
    float Y_act=l*dist_in_img/fx;
    //cout<<"Y: "<<Y_act<<endl;
  
    m.range=sqrt(X_act*X_act+Y_act*Y_act);
    
    if(mc[0].x<ic.x)
       m.heading_angle=atan2(Y_act,X_act)*180/M_PI;
    else
	m.heading_angle=atan2(-Y_act,X_act)*180/M_PI;
    
  }
  imshow(WindowName, img);
  return m;
}
//-----------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle n;

  //-----------------------------------------------------------------------------------------------------------------------------------------
   VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
 
  if (!stream1.isOpened()) { //check if video device has been initialised
    cout << "cannot open camera"<<endl;
    return -1;
  }

  namedWindow(WindowName,WINDOW_NORMAL);
  resizeWindow(WindowName,400,600);
  //-----------------------------------------------------------------------------------------------------------------------------------------

  ros::Publisher chatter_pub = n.advertise<vision::CamMsg>("chatter", 1000);
  //ros::Subscriber flow_xyz = n.subscribe<geometry_msgs::PoseStamped>("mavros//pose", 10, xyz_read);
  ros::Subscriber flow_xyz = n.subscribe("/mavros/px4flow/raw/optical_flow_rad", 10, px4terminal);
  ros::Rate loop_rate(100);
  //ros::spinOnce();

  int count = 0;
  while (ros::ok())
  {
    vision::CamMsg msg;
    msg.range=NAN;
    msg.heading_angle=NAN;

    //-----------------------------------------------------------------------------------------------------------------------------------------
    Mat img;
    stream1.read(img);
    cam_msg m=Process(img);
    msg.range=m.range;
    msg.heading_angle=m.heading_angle;
    //-----------------------------------------------------------------------------------------------------------------------------------------

    ROS_INFO("%f %f",msg.range, msg.heading_angle);
    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

    //-----------------------------------------------------------------------------------------------------------------------------------------
    if (waitKey(30) >= 0)
      break;
    //-----------------------------------------------------------------------------------------------------------------------------------------
  }

  return 0;
}

