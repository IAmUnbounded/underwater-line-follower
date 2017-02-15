#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include<iostream>
#include<math.h>
#include<opencv/highgui.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<math.h>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;
//#include <QCoreApplication>
void uart(char ab,int a,int co){

    int USB = open( "/dev/ttyS5", O_RDWR| O_NOCTTY | O_SYNC );
	int u=1;
    //icout<<USB<<endl;
	char c='A';
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);
   
    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 ) {
       std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }
   
    /* Save old tty parameters */
    tty_old = tty;
   
    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B38400);
    cfsetispeed (&tty, (speed_t)B38400);
   
    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
   
    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  1;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
   
    /* Make raw */
    cfmakeraw(&tty);
   
    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
       std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
if(u==1){
     char cmd[4];
	//cv::waitKey(400);
	sprintf(cmd,"%d",a);
	int n=strlen(cmd);
	//cout<<n<<endl;
	if(n==1){
		cmd[1]=cmd[0];
		cmd[0]='0';
		cmd[2]='\0';
	}
     int spot = 0;
	//write(USB,'s',1);
	if(ab=='e')
	write(USB,&ab,1);
else if(ab!='e'){
	write(USB,&c,1);
	cout<<co<<endl;
	if(co==3){
	write(USB,&ab,1);
    do {
         write( USB, &cmd[spot], 1 );
        spot ++;
    } while (spot<2);
	//cv::waitKey(2000);
    spot=0;
}
}


	do {
         char buf[80];
        int rdlen;

        rdlen = read(USB, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
            buf[rdlen] = 0;
            printf("Read %d: \"%s\"\n", rdlen, buf);
	if(strcmp(buf,"s")==0)
		u=0;
	else if(strcmp(buf,"b")==0){
		u=1;
		break;
	}
	
	}
 else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        }
    } while (1);

}
	


}

int main()
{
   // QCoreApplication a();
    Mat src,dst, color_dst;
	int co=0;
	
    VideoCapture cap(1);
     if(!cap.isOpened())  // check if we succeeded
     {
        cout<<"Video feed not available";
        cout<<"\nCheck for camera availability";
        int n;
        cin>>n;
        return(-1);}
	int s=0;
	double check=0;

  for(;;){
      double angle = 0;
	//double check=0;
  cap>>src;

   //src=imread("/home/sukhad/abcd.jpg");
    resize(src,src,Size(640,480));

    cv::waitKey(30);
       if(!src.data)
            return -1;
   // u_int8_t* pixels = (uchar*)(src.data);
cvtColor(src,src,CV_BGR2HSV);
 inRange(src, Scalar(0, 80, 0), Scalar(179, 255, 255), src);
 namedWindow("Source1",1);
 imshow("Source1",src);
 Canny(src,dst, 100, 300, 3 );

    vector<Point>Largest_contour;
            double max_area=0;
    vector<vector<Point> > contours;
    findContours(dst,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    //cout<<1<<endl;
      //  if(contours.size()==0)
        //            continue;
        //cout<<2<<endl;
              for(unsigned int i=0; i < contours.size(); i++)
               {
                   double area=contourArea(contours[i]);
                   if(area>max_area)
                   {
                       max_area=area;
                       Largest_contour=contours[i];
                   }
               }
               vector<vector<Point> > tcontour;
               tcontour.push_back(Largest_contour);
                           drawContours(dst,tcontour,-1,Scalar(255,0,0),2);

    cvtColor( dst,color_dst, CV_GRAY2BGR );
    namedWindow( "Source", 1 );
       imshow( "Source", dst );

   vector<Vec4i> lines;
    int l,max=0,s,li;
    //cout<<2<<endl;
     //cout<<3<<endl;
    //cout<<lines.size()<<endl;
    HoughLinesP( dst, lines, 1, CV_PI/180,80, 60, 0 );
   if(lines.size()==0){
          // cout<<2<<endl;
           continue;
          }

    //cout<<lines.size()<<endl;
     for(unsigned int i=0;i<lines.size();i++){
            s=(lines[i][2]-lines[i][0])*(lines[i][2]-lines[i][0])+(lines[i][3]-lines[i][1])*(lines[i][3]-lines[i][1]);
            l=sqrt(s);
            if(l>max){
                    max=l;
                    li=i;
            }
        }
     //cout<<lines.size();
    //cout<<2<<endl;
       line( color_dst, Point(lines[li][0], lines[li][1]),
           Point(lines[li][2], lines[li][3]), Scalar(0,0,255), 3, 8 );

		angle=0;
            angle = atan2((double)lines[li][3] - lines[li][1],
                            (double)lines[li][2] - lines[li][0])-angle;
		

   
     namedWindow( "Source", 1 );
        imshow( "Source", dst );

    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", color_dst );


    //std::cout << angle * 180 / CV_PI<< std::endl;
	angle=angle*180/CV_PI;
	if(angle>0){//left
		angle=90-angle;
			if(angle>20){
			if(co==0)
				check=angle;
			co++;
			if(check-angle<20||angle-check<20)
			//	uart('-',angle,co);
			if(co==3){
				check=0;
				co=0;
			}
		}
		else{	co=0; 
			//uart('e',angle,co);
}
	
	}
	else if(angle<0){//right
		angle=angle+90;
		if(angle>20){
			if(co==0)
				check=angle;
			co++;
			if(check-angle<20||angle-check<20)
			//uart('+',angle,co);
			if(co==3){
				check=0;
				co=0;
			}
		}
		else {
			co=0;
			//uart('e',angle,co);
		}	
	}
		
	cout<<angle<<endl;
	 //uart(angle);
    //cv::waitKey(100);
    }

    return 0;
}

