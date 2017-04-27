/*********************************************************** 
*  --- OpenSURF ---                                        *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

/***********************************************************
*  translated by Mr.Hu(Graduate Student of SWJTU)	   *
*  His E-mail is eleftheria@163.com		           *
************************************************************/

#include "surflib.h"
#include "kmeans.h"
#include <ctime>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "capture.hpp"
using namespace std;
//-------------------------------------------------------
//Ϊ\B7\BD\B1\E3\C4\FA\C4\DCʹ\D3\C3OpenSURF,\B6\D4\C4\E3\C4\DC\D7\F6һЩ\BC򵥵\C4\C8\CE\CE\F1\D7\F6\C8\E7\CF\C2˵\C3\F7\A1\A3
//\C4\FAֻ\C4\DCʹ\D3\C3\C6\E4\D6\D0һ\B8\F6\BA\AF\CA\FD\C0\B4ʵ\CF\D6SURF\CC\D8\D5\F7\CC\E1ȡ!
//ͨ\B9\FD\BA궨\D2\E5PROCEDURE\B5\C4ֵ\C0\B4\C3\F7ȷ\A3\BA
//  - 1 \D4\DA\CCṩ\B5\C4·\BE\B6\B5ľ\B2̬ͼƬ\C9\CFʹ\D3\C3
//  - 2 \B4\D3\CD\F8\C2\E7\C9\E3\CF\F1ͷ\BB\F1ȡ
//  - 3 \D4\DAͼƬ\D6\D0\D5ҵ\BDƥ\C5\E4\B5\C4Ŀ\B1\EA
//  - 4 \CF\D4ʾ\D2ƶ\AF\CC\D8\D5\F7
//  - 5 \CF\D4ʾ\BE\B2̬ͼ\CF\F1֮\BC\E4\B5\C4ƥ\C5\E4
#define PROCEDURE 5

//-------------------------------------------------------
//  - 1 \D4\DA\CCṩ\B5\C4·\BE\B6\B5ľ\B2̬ͼƬ\C9\CFʹ\D3\C3
int mainImage(void)
{
  //\C9\F9\C3\F7\C3\E8\CA\F6Ipoints\B5\C4\CF\F2\C1\BF\A3\A8Ipoints=Interest points\A3\AC\BC\B4\A3\BA\D0\CBȤ\B5\E3/\B9ؼ\FC\B5\E3/\CC\D8\D5\F7\B5㣩
  IpVec ipts;
  IplImage *img=cvLoadImage("imgs/sf.jpg");//\D4\D8\C8\EBͼƬ\A3\A8\BE\B2̬ͼ\A3\A9

  //\BC\EC\B2\E2ͼ\CF\F1\D6е\C4\CC\D8\D5\F7\B5㣬\C6\E4\CC\D8\D5\F7\C3\E8\CA\F6\B4\E6\B7\C5\D3\DA\CF\F2\C1\BFipts\D6\D0
  clock_t start = clock();//\BC\C6ʱ\BF\AAʼ
  surfDetDes(img, ipts, false, 5, 4, 2, 0.0004f); //surf\CC\D8\D5\F7\B5\E3\BC\EC\B2\E2
  clock_t end = clock();//ֹͣ\BC\C6ʱ

  std::cout<< "OpenSURF found: " << ipts.size() << " interest points" << std::endl;//\B4\F2ӡ\B3\F6\D5ҵ\BD\B5\C4\CC\D8\D5\F7\B5\E3\B8\F6\CA\FD
  std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;//\B4\F2ӡ\CB㷨\D3\C3ʱ

  // \BB\AD\B3\F6\BC\EC\B2⵽\B5\C4\CC\D8\D5\F7\B5\E3
  drawIpoints(img, ipts);
  
  // \CF\D4ʾ\BD\E1\B9\FB
  showImage(img);

  return 0;
}

//-------------------------------------------------------
//  - 2 \B4\D3\CD\F8\C2\E7\C9\E3\CF\F1ͷ\BB\F1ȡ
int mainVideo(void)
{
  //\B3\F5ʼ\BB\AF\C9\E3\CF\F1ͷ\B2\B6׽\C9豸
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  // \B3\F5ʼ\BB\AF\CA\D3Ƶд\C8\EB
  //cv::VideoWriter vw("c:\\out.avi", CV_FOURCC('D','I','V','X'),10,cvSize(320,240),1);
  //vw << img;

  //\B4\B4\BD\A8һ\B8\F6\B4\B0\BF\DA 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  //\C9\F9\C3\F7\CC\D8\D5\F7\CF\F2\C1\BF
  IpVec ipts;
  IplImage *img=NULL;

  //\D6\F7ѭ\BB\B7
  while( 1 ) 
  {
    //\B2\B6׽һ֡\BB\AD\C3\E6
    img = cvQueryFrame(capture);

    //\CC\E1ȡsurf\CC\D8\D5\F7\B5\E3
    surfDetDes(img, ipts, false, 4, 4, 2, 0.004f);    

    //\BB\AD\B3\F6\BC\EC\B2⵽\B5\C4\CC\D8\D5\F7
    drawIpoints(img, ipts);

    //\BB\E6\D6\C6֡ͼƬ
    drawFPS(img);

    //\CF\D4ʾ\BD\E1\B9\FB
    cvShowImage("OpenSURF", img);

    //\B0\B4Esc\BC\FC\BD\E1\CA\F8ѭ\BB\B7
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------

//  - 3 \D4\DAͼƬ\D6\D0\D5ҵ\BDƥ\C5\E4\B5\C4Ŀ\B1\EA
int mainMatch(void)
{
  //\B3\F5ʼ\BB\AF\CA\D3Ƶ\B2\B6\BB\F1\C9豸
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  //\C9\F9\C3\F7\CC\D8\D5\F7\B5\E3\CF\F2\C1\BF
   
  IpPairVec matches;//ʢ\B7ŵ\C4\CAǵ\E3\B6\D4
  IpVec ipts, ref_ipts;
  
  //object.jpg\CA\C7\CE\D2\C3\C7ϣ\CD\FB\D4\DA\CA\D3Ƶ֡\D6\D0Ѱ\D5ҵĲο\BCĿ\B1\EA
  IplImage *img = cvLoadImage("imgs/object.jpg"); //\D4\D8\C8\EB\B2ο\BCͼ\CF\F1
  if (img == NULL) error("Need to load reference image in order to run matching procedure");
  CvPoint src_corners[4] = {{0,0}, {img->width,0}, {img->width, img->height}, {0, img->height}};
  CvPoint dst_corners[4];

  //\CC\E1ȡ\B2ο\BC\B6\D4\CF\F3\B5\C4\CC\D8\D5\F7\B5\E3\CF\F2\C1\BF
  surfDetDes(img, ref_ipts, false, 3, 4, 3, 0.004f);
  drawIpoints(img, ref_ipts);
  showImage(img);

  //\B4\B4\BD\A8\B4\B0\BF\DA 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  //\D6\F7ѭ\BB\B7\B2\B6\BB\F1
  while( true ) 
  {
    //\B2\B6\BB\F1һ֡\BB\AD\C3\E6
    img = cvQueryFrame(capture);
     
    //\CC\E1ȡ֡\BB\AD\C3\E6\D6е\C4\CC\D8\D5\F7\B5\E3\CF\F2\C1\BF
    surfDetDes(img, ipts, false, 3, 4, 3, 0.004f);

    //\BD\F8\D0\D0\CF\F2\C1\BFƥ\C5\E4
    getMatches(ipts,ref_ipts,matches,640.0,480.0);

	//Ŀ\B1\EA\D4\DA\CA\D3Ƶ֡ͼ\CF\F1\D6е\C4λ\D6\C3
   if (translateCorners(matches, src_corners, dst_corners))
    {
	  //\D4\DAĿ\B1\EA\C9ϻ\AD\BF\F2
      for(int i = 0; i < 4; i++ )
      {
        CvPoint r1 = dst_corners[i%4];
        CvPoint r2 = dst_corners[(i+1)%4];
        cvLine( img, cvPoint(r1.x, r1.y),
          cvPoint(r2.x, r2.y), cvScalar(255,255,255), 3 );
      }

      for (unsigned int i = 0; i < matches.size(); ++i)
        drawIpoint(img, matches[i].first);
    }

    //\BB\E6\D6\C6֡ͼƬ
    drawFPS(img);

    //\CF\D4ʾ\BD\E1\B9\FB
    cvShowImage("OpenSURF", img);

    //\B0\B4Esc\BC\FC\BD\E1\CA\F8ѭ\BB\B7
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}


//-------------------------------------------------------

//  - 4 \CF\D4ʾ\D2ƶ\AF\CC\D8\D5\F7
int mainMotionPoints(void)
{
  //\B3\F5ʼ\BB\AF\CA\D3Ƶ\B2\B6\BB\F1\C9豸
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if(!capture) error("No Capture");

  //\B4\B4\BD\A8һ\B8\F6\B4\B0\BF\DA 
  cvNamedWindow("OpenSURF", CV_WINDOW_AUTOSIZE );

  //\C9\F9\C3\F7Ipoints\BA\CD\C6\E4\CB\FB\B6\AB\CE\F7
  IpVec ipts, old_ipts, motion;
  IpPairVec matches;
  IplImage *img;

  //
  while( 1 ) 
  {
	//ץȡ\B2\B6\BB\F1Դ\B5\C4֡
    img = cvQueryFrame(capture);

    //\BC\EC\B2\E2\BA\CD\C3\E8\CA\F6һ֡\BB\AD\C3\E6\B5\C4Ipoints
    old_ipts = ipts;
    surfDetDes(img, ipts, true, 3, 4, 2, 0.0004f);

    //\BD\F8\D0\D0ƥ\C5\E4
    getMatches(ipts,old_ipts,matches,640.0,480.0);
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {
      const float & dx = matches[i].first.dx;
      const float & dy = matches[i].first.dy;
      float speed = sqrt(dx*dx+dy*dy);
      if (speed > 5 && speed < 30) 
        drawIpoint(img, matches[i].first, 3);
    }
        
    //\CF\D4ʾ\BD\E1\B9\FB
    cvShowImage("OpenSURF", img);

    //\B0\B4Esc\BC\FC\BD\E1\CA\F8ѭ\BB\B7
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  //\CAͷŲ\B6\BB\F1\C9豸
  cvReleaseCapture( &capture );
  cvDestroyWindow( "OpenSURF" );
  return 0;
}







//-------------------------------------------------------
//  - 5 \CF\D4ʾ\BE\B2̬ͼ\CF\F1֮\BC\E4\B5\C4ƥ\C5\E4
int mainStaticMatch()
{
  capture ca;

/*int row=0,col=0;
while(cin>>row>>col)
{
vector<float> hang(col,0);
vector<vector<float> > re(row,hang);
for(int i=0;i<row;i++)
  for(int j=0;j<col;j++)
    {
cin>>re[i][j];


   }
for(int i=0;i<row;i++)
{
  for(int j=0;j<col;j++)
   cout<<re[i][j]<<"  ";
cout<<endl;
}
vector<vector<float> > ree= ca.calEssentialMatrix(  re);
for(int i=0;i<row;i++)
{
  for(int j=0;j<col;j++)
   cout<<ree[i][j]<<"  ";
cout<<endl;
}
}//while
*///\C9\CF\CA\F6\9Cyԇ\D6\F7Ԫ\CB\D8\CF\FBԪ\B0l\CAǿ\C9\D2\D4\D3õ\C4


  IplImage *img1, *img2;
  cv::Mat depth1=imread("imgs/depth1.png" ,-1);

  cv::Mat depth2=imread("imgs/rgb-depth/2/depthImga.png" ,CV_LOAD_IMAGE_UNCHANGED);//讀出圖像的像素值
  cout<<depth1.rows<<"  "<<depth1.cols<<endl;//行480 列640
 /* for(size_t r=0;r<depth2.rows;r++)
  {
    const ushort* rowPtr = depth2.ptr<ushort>(r);
       for(size_t c=0;c<depth2.cols;c++)
          {
            ushort pixel=rowPtr[c];
            if(pixel!=0)
               cout<<(float)pixel/1000.0<<"  ";//scale 的範圍是10000
           //if((int)(uchar)rowPtr[c]!=0.0)
            //cout<<(int)(uchar)rowPtr[c]<<"    ";
       }
  }
 */  
                    
                   
                     
  img1 = cvLoadImage("imgs/rgb1.png");//img1 \D7\F7\A0\91\BB\F9\9CʈDƬ
  img2 = cvLoadImage("imgs/rgb2.png");//\EBS\95r\AB@ȡ\B5\BD\B5ĈDƬ
  //depth1= cvLoadImage("imgs/depth1.png");
  IpVec ipts1, ipts2;
  clock_t start = clock();//\BC\C6ʱ\BF\AAʼ  ̽\B2\E2\BA\CDƥ\C5\E4\B5Ĺ\FD\B3\CC

  surfDetDes(img1,ipts1,false,4,4,2,0.0001f);//\B4\FD\BC\EC\B2\E2\B5\C4ͼ\CF񡢴\E6\B7\C5\CC\D8\D5\F7\B5\E3\B5\C4\CF\F2\C1\BF\A1\A2\CAǷ\F1\D4\DA\D0\FDת\B2\BB\B1\E4\D0ε\C4ģʽ\CF\C2\D4\CB\D0С\A2\BD\F0\D7\D6\CB\FE\B5\C4\D7\E9\CA\FD\A1\A2\BD\F0\D7\D6\CB\FE\D6\D0ÿ\D7\E9ͼ\CF\F1\B5\C4\D7\E9\CA\FD\A1\A2\B3\F5ʼ\B3\E9\D1\F9\A1\A2\E3\D0ֵ\CF\ECӦ surflib.h
  surfDetDes(img2,ipts2,false,4,4,2,0.0001f);//
   
  
  IpPairVec matches;
  getMatches(ipts1,ipts2,matches,640.0,480.0);//\CC\D8\D5\F7\B5\E3ƥ\C5䣬\B1\EDʾ\B3\F6\CF\F2\C1\BF֮\BC\E4\B5\C4ƥ\C5\E4,\B8\F8\B3\F6\C1\CBƥ\C5\E4\B5ĵ\E3\B6Ժ\CDƥ\C5\E4\B5ķ\BD\CF\F2
  clock_t end = clock();//ֹͣ\BC\C6ʱ  ̽\B2\E2\BA\CDƥ\C5\E4\BD\E1\CA\F8\B5Ĺ\FD\B3\CC
   

    
    // num[0]='a';
/*ð\C5\DD\C5\C5\D0\F2*/
 

  //int size=matches.size();//  ca.bubbleSort(matches,2,4);//matches \D1Y\C3\E6ʢ\B7ŵ\C4\CA\C7\CB\F9\D3е\C4\CC\D8\E1\E7\FCc首先將所有的對應到一個點上的特徵點去掉
  int len3= ca.bubbleSort(matches,2,4);//選擇之後的數據
  IpPairVec  re=matches;
  cout<<"len3="<<len3<<endl; 
  for(int i=re.size();i>len3;i--)
    re.pop_back();//
  cout<<"after  re.size()="<<re.size()<<endl;

/*
CvFont font; 
    double hscale = 0.51;
    double vscale = 0.5;
    int linewidth = 0.5;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX |CV_FONT_ITALIC,hscale,vscale,0,linewidth);

    CvScalar textColor =cvScalar(0,0,255);
    char num='a';


for (unsigned int i = 0; i < len3; ++i)//matches.size()
  { //cout<<"***************************"<<endl;
    // if((re[i].first.orientation-re[i].second.orientation)<0.2)
    // {
     
    drawPoint(img1,re[i].first);
    drawPoint(img2,re[i].second);
    const int & w = img1->width;//height;
    
    cvLine(img1,cvPoint(re[i].first.x,re[i].first.y),cvPoint(re[i].second.x+w,re[i].second.y), cvScalar(255,255,255),1);//ʹ\D3\C3w\B6Է\BD\CF\F2\BD\F8\D0\D0\C1\CB\C9\E8\D6\C3
    cvLine(img2,cvPoint(re[i].first.x-w,re[i].first.y),cvPoint(re[i].second.x,re[i].second.y), cvScalar(255,255,255),1);
     
    //const char * text=num;
    CvPoint textPos =cvPoint(re[i].first.x,re[i].first.y);
    cvPutText(img1, &num, textPos, &font,textColor);
    CvPoint textPos2 =cvPoint(re[i].second.x,re[i].second.y);
    cvPutText(img2, &num, textPos2, &font,textColor);
    num++;
    //sleep(1);
 
//}

}//for

*/

/*good matches \D2ѽ\9B\B5õ\BD*/
vector<ushort> hang(640.0,0);
vector<vector<ushort> > depth(480.0,hang);




 
 /*for( int y = 0; y < depth1.rows; y++ ) {
 
   for( int x = 0; x < depth1.cols; x++ ) 
{
         ushort d = depth1.ptr<ushort>( int(y) )[ int(x) ]; 
 if(d!=0)
         cout<<d<<"    ";
         depth[y][x]=d;
        
 }

} */
/*for(int i=0;i<480;i++)
  for(int j=0;j<640;j++)
   {CvScalar cs=cvGet2D(depth1, i, j) ; 
     
      depth[i][j]=depth1[i][j] ;
 cout<<depth[i][j]<<"  ";
}*/



cout<<"test"<<endl;

int t=matches.size();
/*以上是發現角點的*/
ca.calRotaAndTran( depth1,matches,/*re,*/ img1,img2,t);//Ӌ\CB\E3\B3\F6\D4u\D7h\BA\CD\D0\FD\DED\B5\C4\D0\C5Ϣc




/*cout<<"re.size()"<<re.size()<<endl;

 
  //int size=matches.size();
  for (unsigned int i = 0; i < size; ++i)//matches.size()
  { 
     //if((matches[i].first.orientation-matches[i].second.orientation)<0.2)
     //{
     //cout<<i<<"   "<<num<<"  "<<matches[i].first.orientation<<"  "<<matches[i].second.orientation<<endl;
     //cout<<i<<"   "<<num<<"  "<<matches[i].first.x<<"    "<<matches[i].first.y<<"   "<<matches[i].first.distance<< endl;
     //cout<<i<<"   "<<num<<"  "<<matches[i].second.x<<"  "<<matches[i].second.y<<endl;
  
    drawPoint(img1,matches[i].first);
    drawPoint(img2,matches[i].second);
    const int & w = img1->width;//height;
    
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);//ʹ\D3\C3w\B6Է\BD\CF\F2\BD\F8\D0\D0\C1\CB\C9\E8\D6\C3
    cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
     
    //const char * text=num;
     CvPoint textPos =cvPoint(matches[i].first.x,matches[i].first.y);
     cvPutText(img1, &num, textPos, &font,textColor);
     CvPoint textPos2 =cvPoint(matches[i].second.x,matches[i].second.y);
     cvPutText(img2, &num, textPos2, &font,textColor);
     num++;
    //sleep(1);
 
//}

} //for
 */

//CAMERA_INTRINSIC_PARAMETERS C;
/*vector<vector<float> >para= ca.parameter(matches,6,13);
int paralen1=para.size();
int paralen2=para[0].size();
// ca.switchMax(para,  0);
for(int i=0;i<paralen1;i++)
 {
  //ca.switchMax(para,  i);
 for(int j=0;j<paralen2;j++)
   cout<<para[i][j]<<"  ";
cout<<endl;
 }
*/

/*vector<vector<float> > para1=ca.calEssentialMatrix(para);


cout<<endl;

for(int i=0;i<paralen1;i++)
 {

 for(int j=0;j<paralen2;j++)
   cout<<para1[i][j]<<"  ";
   cout<<endl;
 }*/


  std::cout<< "Matches: " << matches.size() <<std::endl;
  std::cout<< "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC  << " seconds" << std::endl;//\B4\F2ӡ\CB㷨\D3\C3ʱ
  
   cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
  cvShowImage("1", img1);
  cvShowImage("2",img2);
  cvWaitKey(0);
   
  return 0;
}//metched \B4惦\B5\C4\CC\D8\E1\E7\FCc\CAǻ\A5\B2\BB\CF\E0ͬ\B5\C4


//-------------------------------------------------------

int mainKmeans(void)
{
  IplImage *img = cvLoadImage("imgs/img1.jpg");
  IpVec ipts;
  Kmeans km;
  
  //\BB\F1ȡ\CC\D8\D5\F7\B5\E3
  surfDetDes(img,ipts,true,3,4,2,0.0006f);

  for (int repeat = 0; repeat < 10; ++repeat)
  {

    IplImage *img = cvLoadImage("imgs/img1.jpg");
    km.Run(&ipts, 5, true);
    drawPoints(img, km.clusters);

    for (unsigned int i = 0; i < ipts.size(); ++i)
    {
      cvLine(img, cvPoint(ipts[i].x,ipts[i].y), cvPoint(km.clusters[ipts[i].clusterIndex].x ,km.clusters[ipts[i].clusterIndex].y),cvScalar(255,255,255));
    }

    showImage(img);
  }

  return 0;
}

//-------------------------------------------------------

int main(void) 
{
  if (PROCEDURE == 1) return mainImage();
  if (PROCEDURE == 2) return mainVideo();
  if (PROCEDURE == 3) return mainMatch();
  if (PROCEDURE == 4) return mainMotionPoints();
  if (PROCEDURE == 5) return mainStaticMatch();
  if (PROCEDURE == 6) return mainKmeans();
}
