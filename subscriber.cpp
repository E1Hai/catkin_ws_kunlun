#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <vector>

std::vector<cv::Point3f> obj;
std::vector<cv::Point2f> img;
cv::Mat  KA(3,3,cv::DataType<double>::type),DI(1,5,cv::DataType<double>::type),rvec(3,3,cv::DataType<double>::type),tvec(3,1,cv::DataType<double>::type);


unsigned long long z[6][2],y[6][2];
// z[][] stores the translation of the 6 search-region centers and y stores the 
// average position of the 6 white markers (answer of 4-4)



double  ka[3][3],di[5];



//***************************************************************************extracting Camera parameters********************************************************
void cb(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
static bool stop =1;
if (stop){  // extract the parameters only one time
                                 ROS_INFO("fx=%f   fy=%f  cx=%f cy=%f \n k1=%f  k2=%f  t1=%f  t2=%f  k3=%f\n\n", 
		                                           msg->K[0],msg->K[4],msg->K[2],msg->K[5],
		                                           msg->D[0],msg->D[1],msg->D[2],msg->D[3],msg->D[4]);


ka[0][0]=msg->K[0];
ka[1][1]=msg->K[4];
ka[0][2]=msg->K[2];
ka[1][2]=msg->K[5];
ka[0][1]=0;
ka[1][0]=0;
ka[2][0]=0;
ka[2][1]=0;
ka[2][2]=1;
di[0]=msg->D[0];
di[1]=msg->D[1];
di[4]=msg->D[4];
di[2]=msg->D[2];
di[3]=msg->D[3];


KA = cv::Mat(3,3,cv::DataType<double>::type,ka);
DI = cv::Mat(1,5,cv::DataType<double>::type,di);

stop =0;
                 }  // end if
}
//******************************************************************************************************************************************************************





static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/sensors/camera/infra1/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }



/*
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(268, 122), 1, CV_RGB(255,0,0)); //1
      cv::circle(cv_ptr->image, cv::Point(416, 112), 1, CV_RGB(255,0,0)); //2
      cv::circle(cv_ptr->image, cv::Point(246, 162), 1, CV_RGB(255,0,0)); //3
      cv::circle(cv_ptr->image, cv::Point(448, 150), 1, CV_RGB(255,0,0)); //4
      cv::circle(cv_ptr->image, cv::Point(206, 238), 1, CV_RGB(255,0,0)); //5
      cv::circle(cv_ptr->image, cv::Point(506, 223), 1, CV_RGB(255,0,0)); //6
*/

  // Threshold-----------------------------------------------------------------------
    threshold(cv_ptr->image, cv_ptr->image,250, 255,cv::THRESH_BINARY); 


//***********************************************************************************************************************************************
static bool stop =1;

if(stop)
{
// initialising 6 search centers for the 6 regions
z[0][0]=268-13;		z[3][0]=448-13;
z[0][1]=120-13;		z[3][1]=150-13;
z[1][0]=416-13;		z[4][0]=206-13;
z[1][1]=110-13;		z[4][1]=238-13;
z[2][0]=245-13;		z[5][0]=506-16;
z[2][1]=160-13;		z[5][1]=223-16;

for(unsigned long long k=0;k<6;++k){ unsigned long long cc=0; // cc is a counter for the white pixels
    for(unsigned long long i =0;i<32;++i){                const unsigned char* row = cv_ptr->image.ptr<unsigned char>(z[k][1]+i);
	  for(unsigned long long j=0;j<32;++j)
	
if( row[z[k][0]+j]  >  0  ) 
{
++cc;
y[k][0]+=(z[k][0]+i);
y[k][1]+=(z[k][1]+j);
}// ******************************end if
}//********************************************end for(i) loop

ROS_INFO("number of calculated  white pixels=%lli",cc);

y[k][0]=y[k][0]/cc;
y[k][1]=y[k][1]/cc;
ROS_INFO("pixel[%llu][0]= %llu   pixel[%llu][1]= %llu",k,y[k][0],k,y[k][1]);

} // ************ end for(k) loop

// **************************************************************************************************initiailsing  imagePoints-matrix
for(int i=0;i<6;++i)  
img.push_back(cv::Point2f(y[i][0],y[i][1]));
//***********************************************************************************************************************************
//
//
//
//
//
//**********************************************************************************************************initialising objectPoints 
obj.push_back(cv::Point3f(0.5, 0.2,  0));
obj.push_back(cv::Point3f(0.5,-0.2,  0));
obj.push_back(cv::Point3f(0.8, 0.2,  0));
obj.push_back(cv::Point3f(0.8,-0.2,  0));
obj.push_back(cv::Point3f(1.1, 0.2,  0));
obj.push_back(cv::Point3f(1.1,-0.2,  0));
//***********************************************************************************************************************************
//
//
//
//*****************************************************************************************************************calling solvePnP and giving the results***
cv::solvePnP(obj,img,KA,DI,rvec,tvec);
//for(int i=0;i<3;++i)
  // for(int j=0;j<3;++j)
// std::    cout << rvec.at<double>(i,j)<<std::endl;
std::cout<<"\n\nrvec = "<<std::endl<<rvec<<std::endl;  
std::cout<<"\n\ntvec = "<<std::endl<<tvec<<std::endl;  
cv::Rodrigues(rvec,rvec);
std::cout<<"\n\nrvec = "<<std::endl<<rvec<<std::endl;  
//
//
//
//
//*******************************************************************************************************************  Assigment 4-6 end  *********************
stop=0;
}
//**********************************************************************************************************************************************

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");
 
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/sensors/camera/infra1/camera_info", 1000, cb); //******Assignment 4-2
  ImageConverter ic;//************************Assignment 4-3
 
 

  ros::spin();

  return 0;

}
