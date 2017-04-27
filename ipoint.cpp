/*********************************************************** 
*  --- OpenSURF ---                                       *
*  This library is distributed under the GNU GPL. Please   *
*  use the contact form at http://www.chrisevansdev.com    *
*  for more information.                                   *
*                                                          *
*  C. Evans, Research Into Robust Visual Features,         *
*  MSc University of Bristol, 2008.                        *
*                                                          *
************************************************************/

#include <cv.h>
#include <vector>

#include "ipoint.h"

//����ƥ�����1��2 ��ʾ�㼯�ϣ�����3��ʾ��Եļ���

float distance(Ipoint first,Ipoint second,float w,float h)
{
float chang=(w-first.x+second.x)*(w-first.x+second.x);
float kuan=(first.y-second.y)*(first.y-second.y);
float dis=sqrt(chang+kuan); 
return dis;
}

 


void getMatches(IpVec &ipts1, IpVec &ipts2, IpPairVec &matches,float w,float h)
{
  float dist, d1, d2;
  Ipoint *match;

  matches.clear();//��һ�������Ͳ���

  for(unsigned int i = 0; i < ipts1.size(); i++) 
  {
    d1 = d2 = FLT_MAX;

    for(unsigned int j = 0; j < ipts2.size(); j++) 
    {
      dist = ipts1[i] - ipts2[j];  //��������أ���ʾsqrt(��������֮��ÿ��Ԫ�ز��ƽ����)

      if(dist<d1) //���������ƥ��ȵ�ǰ���ƥ�仹��
      {
        d2 = d1;
        d1 = dist;
        match = &ipts2[j];
      }
      else if(dist<d2) //���������ƥ��ȵ�ǰ�μ�ƥ�仹��
      {
        d2 = dist;
      }//�ҵ���d1,d2����ɵ�����
    }

	//���d1:d2<0.65����Ϊ��һƥ���
    if(d1/d2 < 0.65) //�������Ҳ��ƥ���ϣ�˵����һ��ƥ��ĵ�
    { 
      //�洢λ�õı仯
      ipts1[i].dx = match->x - ipts1[i].x; //��ֹ���ȥ��ʼ���ʾ����һ������
      ipts1[i].dy = match->y - ipts1[i].y;
      ipts1[i].distance= distance( ipts1[i], *match,  w,  h);
      match->distance =ipts1[i].distance;
      matches.push_back(std::make_pair(ipts1[i], *match));//ǰ�߱�ʾ���򣬺��߱�ʾ�õ�ƥ��ĵ㣬ͨ���������������Ա�ʾ����ʼ��

    }
  }
}

//
//��Ӧ�ԣ������Ϊ��͸�ӱ任��ʹ�õ���CV_RANSAC������OpenCV 1.1�����ڴ����linux���а��в������
//�Ӷ�άƽ���ϵ�һ���㵽�������ϵĵ�
//-------------------------------------------------------

//���ƥ���֮��ĵ�Ӧ�Ծ��󣬲���src_corners�任��dst_corners
int translateCorners(IpPairVec &matches, const CvPoint src_corners[4], CvPoint dst_corners[4])
{
#ifndef LINUX
  double h[9];//���浥Ӧ�Ծ��������h
  CvMat _h = cvMat(3, 3, CV_64F, h);//��hת���ɾ���
  std::vector<CvPoint2D32f> pt1, pt2;
  CvMat _pt1, _pt2;
  
  int n = (int)matches.size();
  if( n < 4 ) return 0;

  //���������Ĵ�С
  pt1.resize(n);
  pt2.resize(n);

  //�����ĸ���
  for(int i = 0; i < n; i++ )
  {
    pt1[i] = cvPoint2D32f(matches[i].second.x, matches[i].second.y);
    pt2[i] = cvPoint2D32f(matches[i].first.x, matches[i].first.y);
  }
  _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
  _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );

  //��������㼯֮��ĵ�Ӧ�Ա任����h
  //if(!cvFindHomography(&_pt1, &_pt2, &_h, CV_RANSAC, 5))  //����Ҫ��opencv 1.1
   // return 0;

  //����������õĵ�Ӧ�Ա任����h����src_corners�任��dst_corners
  for(int i = 0; i < 4; i++ )
  {
    double x = src_corners[i].x, y = src_corners[i].y;
    double Z = 1./(h[6]*x + h[7]*y + h[8]);
    double X = (h[0]*x + h[1]*y + h[2])*Z;
    double Y = (h[3]*x + h[4]*y + h[5])*Z;
    dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
  }
#endif
  return 1;
}


