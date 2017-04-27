/*相機位姿求解，齊次座標求解，*/
#include<iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;
const float cpatureMAX=1367.05;//最大值是兩幅圖像的斜對角線即（640+640）×（640+640）+480*480的根下值
const float captureMIN=0.0;//規定一個最小值，輸出最大的距離值
class capture
{
public:
 
struct CAMERA_INTRINSIC_PARAMETERS 
 { 
      double cx, cy, fx, fy, scale;
  };

/*size作爲內參傳入，因爲matches的內部參數有冗餘的*/
pair<float,float> minAndMaxDistance(IpPairVec matches,int size)
{
pair<float,float> minAndMaxDis(0.0,0.0);

float minDistance=cpatureMAX;//規定了一個最大值，
float maxDistance=captureMIN;
for(int i=0;i<size;i++)
{
if(matches[i].first.distance<minDistance)
   
    minDistance=matches[i].first.distance;     
if(matches[i].first.distance>maxDistance)
   
    maxDistance=matches[i].first.distance;           
          
   

minAndMaxDis=pair<float,float>(minDistance,maxDistance);

}


return minAndMaxDis;//返回距離的最小值
}




IpPairVec select(IpPairVec matches,int size)
{//從中選出
IpPairVec re; 
pair<float,float> minAndMaxDis=minAndMaxDistance(matches,size);//最大最小的距離
cout<<" miniDistance="<< minAndMaxDis.first<<"maxDiatance="<<minAndMaxDis.second<<"  "<<minAndMaxDis.first+(minAndMaxDis.second-minAndMaxDis.first)/4.5<<endl;

for(int i=0;i<size;i++)
 {
if(matches[i].first.distance>=minAndMaxDis.first  &&minAndMaxDis.first+(minAndMaxDis.second-minAndMaxDis.first)/4.5>= matches[i].first.distance&&i%2==0)
//if(matches[i].first.distance<664)//644可以寫到配置文件當中

re.push_back(matches[i]);
  }
return re;
}



 int  bubbleSort(IpPairVec &matches,const int num,const int num0)//第二個參數表示使用first還是second權值
{//冒泡排序，從小到大進行排序、去重  
 int len= matches.size();
 bool flag=true;
 for(int i=0;i<len;i++)
   {
     if(flag==false) 
       break;
     flag=false;
     for(int j=0;j<len-i-1;j++)
       {
       if(num==1&&matches[j].first.orientation>matches[j+1].first.orientation)
       {
        pair<Ipoint,Ipoint> swap=matches[j] ;//要將整個元素進行交換
        matches[j] =matches[j+1] ;
        matches[j+1] =swap;
        flag=true;

        } 
        if(num==2&&matches[j].second.orientation>matches[j+1].second.orientation)
       {//按照第二個權值進行排序 
        pair<Ipoint,Ipoint> swap=matches[j] ;
        matches[j] =matches[j+1] ;
        matches[j+1] =swap;
        flag=true;

        }         

       }//for(j)

  }//for(i)

  int t=0;
  int count=0;//去掉重複的點：如果重複說明詞典和另外的幾個點都可以匹配，說明是錯誤的
 vector<vector<pair<int,int> > > re;//表示重複的對應的點
 
 flag=true;
 int size;



  if(num0==1){
 for(int i=1;i<len-1;i++)
  {
     if(matches[t].first!=matches[i].first&&flag==false)
       {
       
         matches[++t]=matches[i];
        // count++; 
        // t++;
        }
     else if(matches[t].first==matches[i].first&&flag==true)
      {
       // t++;
        flag=false;
      }
   //相等但等於false的時候，不用加一 
     else if(matches[t].first!=matches[i].first&&flag==true)
        t++;

  }//for(i)
 


}


else if(num0==2){

for(int i=1;i<len-1;i++)
  {
     if(matches[t].second!=matches[i].second&&flag==false)
       {
       
         matches[++t]=matches[i];
         count++; 
        // t++;
        }
     else if(matches[t].second==matches[i].second&&flag==true)
      {
      //  t++;
        flag=false;
      }
   //*相等但等於false的時候，不用加一 
     else if(matches[t].second!=matches[i].second&&flag==true)//位置和方向有不相同的時候
        t++;

  }//for(i)


} 

else if(num0==3)
{//去掉所有的重複的，一個都不留下
t=0;
int i=1;
while(i<len)
{
  while(i<len&&matches[t].first!=matches[i].first)
{
   matches[++t]=matches[i];
    
   i++;
}

if(i<len)
  {
while(i<len&&matches[t].first==matches[i].first)
 i++;
 }
if(i<len)
  {
   matches[t]=matches[i];
   i++;
}
} //while



if(matches[len-2].first==matches[len-1].first )
 t-=1;


}//while(i)

else if(num0==4)
{//去掉所有的重複的，一個都不留下
t=0;
int i=1;
while(i<len)
{
  while(i<len&&matches[t].second!=matches[i].second)
{
   matches[++t]=matches[i];
    
   i++;
}

if(i<len)
  {
while(i<len&&matches[t].second==matches[i].second)
 i++;
 }
if(i<len)
  {
   matches[t]=matches[i];
   i++;
}
} 




if(matches[len-2].second==matches[len-1].second)
 t-=1;




}
 

return t+1;//表示當時matches的大小
 
//return len;
  }//end;
//此案函數是構造
//判斷一個數組元素是否都是0
bool judgeVecIsZero(vector<float> hang)
{
int len=hang.size();
if(len==0) return true;
int i=0;
while(i<len)
{
if(hang[i]!=0.0) return false;
}
return true;//

}
//判斷兩個向糧是否相等
bool judgeEqual(vector<float> hang1,vector<float> hang2)
{ 

  if( judgeVecIsZero( hang1)&& judgeVecIsZero(hang2)) return true;
  int  len1=hang1.size();
  int  len2=hang2.size();
  if(len1!=len2) return false;  
  int i=len1;
  float div=0.0;
  int count=0;
  while(i>=0)
{
    if(hang1[i]==hang2[i]||(hang1[i]==0&&hang2[i]==0))
      i--;
    else if((hang1[i]==0&&hang2[i]!=0)||(hang1[i]!=0&&hang2[i]==0)) 
      return false;//如果其中有一個數爲0，而另一個值不爲0
    else if(hang1[i]!=hang2[i]&&count==0)
    {
     div=hang1[i]/hang2[i]; 
     count++;
     i--;
    }

else if(hang1[i]!=hang2[i]&&count!=0)
{
  if(hang1[i]/hang2[i]!=div)  
     return false;

}

}//while(i)
return true;
}

//matches特徵點對的集合
//matches集合的大小
//row和rol指的是行列式的行和列
//begin和end是在matches中選取元素的開始和終止位置
vector<vector<float> > parameter(IpPairVec matches,int begin,int end)
{
  vector<vector<float> > re;  
  int size=matches.size();
  if(size<8) return re;//取九個點可以不管另外自由度的約束進行求解
  for(int i=begin;i<=end;i++)//表示的是行列式的行數，其中end-begin=row 
   {
     vector<float> hang;
     float firstx=(matches[i].first.x-959.5)/1081.37;
     float firsty=(matches[i].first.y-539.5)/1081.37;
     float secondx=(matches[i].second.x-959.5)/1081.37;
     float secondy=(matches[i].second.y-539.5)/1081.37;
     hang.push_back((float) firstx*(float) secondx);
     hang.push_back((float) firstx*(float) secondy);
     hang.push_back((float) firstx);
     hang.push_back((float) firsty*(float)  secondx);
     hang.push_back((float) firsty*(float) secondy);
     hang.push_back((float) firsty);
     hang.push_back((float) secondx);
     hang.push_back((float) secondy);
     hang.push_back(1.0);
     re.push_back(hang);
     }//for(i) 
     return re;//返回參數矩陣
}


vector<vector<int> > parameter1(IpPairVec matches,int begin,int end)
{
  vector<vector<int> > re;  
  int size=matches.size();
  if(size<8) return re;//取九個點可以不管另外自由度的約束進行求解
  for(int i=begin;i<=end;i++)//表示的是行列式的行數，其中end-begin=row 
   {
     vector<int> hang;


     hang.push_back((int)matches[i].first.x*(int)matches[i].second.x);
     hang.push_back((int)matches[i].first.x*(int)matches[i].second.y);
     hang.push_back((int)matches[i].first.x);
     hang.push_back((int)matches[i].first.y*(int)matches[i].second.x);
     hang.push_back((int)matches[i].first.y*(int)matches[i].second.y);
     hang.push_back((int)matches[i].first.y);
     hang.push_back((int)matches[i].second.x);
     hang.push_back((int)matches[i].second.y);
     hang.push_back(1);
     re.push_back(hang);
     }//for(i) 
     return re;//返回參數矩陣
}



//交換使當前行的第一個元素不爲0，作爲基準元素進行下面的計算，交換是必須要進行的 

bool  switchVec(vector<vector<float> > &re,int i)
{
int j=i+1;
int len=re.size();
while(j<len)
{
if(re[j][i]!=0)
 {//交換兩個向量
vector<float> swap=re[i];
re[i]=re[j];
re[j]=swap;//jiaohuanglianghang
j++;
return true;
}//if
}//while

return false;//如果都等於0咋進行下一個階段的計算
}

bool  switchVec(vector<vector<int> > &re,int i)
{
int j=i+1;
int len=re.size();
while(j<len)
{
if(re[j][i]!=0)
 {//交換兩個向量
vector<int> swap=re[i];
re[i]=re[j];
re[j]=swap;//jiaohuanglianghang
j++;
return true;
}//if
}//while

return false;//如果都等於0咋進行下一個階段的計算
}


void  switchMax(vector<vector<float> > &re,int i)
{
int len=re.size();
float max=abs(re[i][i]);
int flag=i;
 for(int j=i;j<len;j++)

{
if(abs(re[j][i])>max)
{
max=abs(re[j][i]);
flag=j;
}
}//for(j) 
vector<float> swap=re[i];
re[i]=re[flag];
re[flag]=swap;//jiaohuanglianghang
return ;
  
}
void  switchMax(vector<vector<int> > &re,int i)
{
int len=re.size();
float max=abs(re[i][i]);
int flag=i;
 for(int j=i;j<len;j++)
{
if(abs(re[j][i])>max)
{
max=abs(re[j][i]);
flag=j;
}
}//for(j) 
vector<int> swap=re[i];
re[i]=re[flag];
re[flag]=swap;//jiaohuanglianghang
return ;
  
}


//參數爲參數矩陣，求解此參數矩陣下，相乘爲0的因子，也就是基本矩陣
//求解齊次線性方程組的解
  vector<vector<float> >  calEssentialMatrix(vector< vector <float> > re)
  {
   //利用已經選出的點對相機的位置進行求解,列主元素消除的方法
   vector<vector<float> > ree=re;
   //int sum=0;
   //vector<vector<float> > es;
   int len=ree.size();//表示整個數組的長度


 
   for(int i=0;i<ree.size() ;i++)
    {

      //switchMax(ree,i);//在相減的時候進行最大值的調整
      int len2=ree[i].size();
      if(ree[i][i]==0)
         { continue;
           bool flag=switchVec(ree,i);//如果這一列當中沒有非0元素
           if(flag==false) continue;//如果這一列都是0了則進行下一回合的計算
         }//使得第i行的第i個元素不爲0方便下面的計算
/*上面保證了ree[i][i]!=0*/
      float div=ree[i][i];
      for(int j=i;j<ree[i].size();j++)
         {
          ree[i][j]/=div;//將第一個數字變爲1，方便以後的計算
         }
      for(int k=0;k<len;k++) 
         {//從ree的第一行開始進行計算,所以k從等於0開始計算
           if(ree[k][i]!=0&&k!=i)//不是當前這一行
            {  

               div=ree[k][i];//表示成乘數因子是第k行的第i個數，這個一行第i個數之後的所有的數減去第i行乘以乘數因子，得到新的k這一行的數值
               for(int t=i;t<len2;t++)//這是第i行，也是璁第i個數開始的
                  ree[k][t]=ree[k][t]-div*ree[i][t];//將每一行減去當前的i行中每個數值的div倍
            }


         }

    }
 return  ree;
  // return es;



}


vector<vector<int> >  calEssentialMatrix(vector< vector <int> > re)
  {
   //利用已經選出的點對相機的位置進行求解,列主元素消除的方法
   vector<vector<int> > ree=re;
   //int sum=0;
   //vector<vector<float> > es;
   int len=ree.size();//表示整個數組的長度


 
   for(int i=0;i<ree.size() ;i++)
    {

      //switchMax(ree,i);//在相減的時候進行最大值的調整
      int len2=ree[i].size();
      if(ree[i][i]==0)
         { continue;
           bool flag=switchVec(ree,i);//如果這一列當中沒有非0元素
           if(flag==false) continue;//如果這一列都是0了則進行下一回合的計算
         }//使得第i行的第i個元素不爲0方便下面的計算
/*上面保證了ree[i][i]!=0*/
      int div=ree[i][i];
      for(int j=i;j<ree[i].size();j++)
         {
          ree[i][j]/=div;//將第一個數字變爲1，方便以後的計算
         }
      for(int k=0;k<len;k++) 
         {//從ree的第一行開始進行計算,所以k從等於0開始計算
           if(ree[k][i]!=0&&k!=i)//不是當前這一行
            {  

               div=ree[k][i];//表示成乘數因子是第k行的第i個數，這個一行第i個數之後的所有的數減去第i行乘以乘數因子，得到新的k這一行的數值
               for(int t=i;t<len2;t++)//這是第i行，也是璁第i個數開始的
                  ree[k][t]=ree[k][t]-div*ree[i][t];//將每一行減去當前的i行中每個數值的div倍
            }


         }

    }
 return  ree;
  // return es;



}


cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
 {//利用2D點轉化到3D點，轉化出來的點作爲顯得點加入到
     cv::Point3f p; // 3D 点
     p.z = double( point.z ) / camera.scale;
     p.x = ( point.x - camera.cx) * p.z / camera.fx;
     p.y = ( point.y - camera.cy) * p.z / camera.fy;//求得這一點在空間中的位置
     return p;
 }

void correctIndex(cv::Mat &inliers,IpPairVec matches,int t)
{
/*優化，捨棄不符合大多數特徵點的*/
 int countXP=0;
 int countXN=0;
 int countYP=0;
 int countYN=0; 
 bool flagX=false;
 bool flagY=false;
for(int j=0;j<inliers.rows;j++)
   {
    const ushort* rowPtr = inliers.ptr<ushort>(j);
    int i=rowPtr[0];
    if(matches[i].first.x>matches[i].second.x)
       countXP++;
    else 
       countXN++;
    if(matches[i].first.y>matches[i].second.y)
        countYP++;
    else countYN++;
   }
if(countXP>countXN) flagX=true;
if(countYP>countYN) flagY=true;
for(int j=0;j<inliers.rows;j++)
{
    const ushort* rowPtr = inliers.ptr<ushort>(j);
    int i=rowPtr[0];
    if((matches[i].first.x>matches[i].second.x&&flagX==false)||(matches[i].first.x<=matches[i].second.x&&flagX==true))
       inliers.at< ushort>(j,0)=(ushort)t;
    if((matches[i].first.y>matches[i].second.y&&flagY==false)||(matches[i].first.y<=matches[i].second.y&&flagY==true))
       inliers.at< ushort>(j,0)=(ushort)t;

}
   


}





void calRotaAndTran(cv::Mat depth1,IpPairVec matches,IplImage *img1,IplImage  *img2,int t)//表示第一幅圖像的深度信息
{   

    // 計算平移和旋轉的矩陣
    // 计算图像间的运动关系
    // 关键函数：cv::solvePnPRansac()
    // 为调用此函数准备必要的参数
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;//盛放有深度的圖像上面的匹配到的點，裏面的點是通過轉換得到的
       
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;//盛放的是2D平面上的點，這些點是在當時隨時得到的圖片上的位置

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS C;//參數有待調整
    C.cx = 316.615;//332.654;//290.098;//325.5;//
    C.cy = 253.189;//259.925;//316.151;//253.5;//
    C.fx = 626.106;//643.896;//524.148;//518.0;//
    C.fy = 632.413;//647.544;//593.189;//519.0;//
    C.scale = 1000.0;//相機的內參，根據實際情況可以進行調整,長度範圍在70釐米左右，所以應該是

    /*for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( kp2[goodMatches[i].trainIdx].pt ) );

         // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, C );
        pts_obj.push_back( pd );
    }*/


    cout<<"matches.size()="<<matches.size()<<"   ";
      
       
    for(size_t i=0; i<matches.size();i++)
    {
    //cout<<"test0"<<endl;
    cv::Point2f p;
    /*匹配的特徵點在圖像上的三維座標*/
    p.x=matches[i].first.x;//表示可以匹配上的點
    p.y=matches[i].first.y;
    ushort d = depth1.ptr<ushort>(int(p.y) )[int( p.x ) ];//取出圖片對應的深度信息，深度值是無符號短整型的數字
  

    if(d==0||d<500||d>2000)
       continue;
    // cout<<d<<"    ";
    //cout<<"test1"<<endl;
    pts_img.push_back(cv::Point2f(matches[i].second.x,matches[i].second.y));//二維圖像上相匹配的點，根據這些點計算評議和旋轉的角度
    cv::Point3f pt(p.x, p.y, d );//這個是第一幅標準圖像的三維座標
    cv::Point3f pd = point2dTo3d( pt, C );//計算出的第二個位置的時候，還是該點的三維座標，這時x y z相對於第一個位置點的座標發生了變化  pd是實際空間座標 
    //cout<<"test2"<<endl;
    pts_obj.push_back( pd );//收入的是3D點的座標，是這個點的空間座標，實際空間座標
    //cout<<"test3"<<endl;
    }
   

    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };
    double dis[1][8]={ 0.188518,-0.739376,0.007555,0.009332,0.027309,0.280883,0.001870,0.001309};//畸變參數
    Mat H(1,8, CV_64F); 
    cout<<"列:"<<H.cols<<endl;
    cout<<"行:"<<H.rows<<endl;
     for(int i = 0; i < H.rows; i++)//表示行
            for(int j = 0; j < H.cols; j++)//表示列
                  {
                   H.at<double>(i,j)=dis[i][j];
                   cout<<dis[i][j]<<"  ";
                   }
    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    //求解pnp，xyz的座標是一圖像中線爲原點的， 實際的空間中的點，經歷了平移旋轉，再乘以攝像頭的內參，得到的是現在次圖像上的
     cout<<"matches.size():"<<matches.size()<<endl;
     cout<<"nide"<<endl;
     //cout<<"inliers: testwode"<<inliers.rows<<endl;//顯示內部點的個數
/*給出目標的一系列的點，和他們相對應的圖像投影、攝像機矩陣 和畸變參數，函數可以找到最小化投影誤差的姿勢，這個最小誤差就是相對應的誤差點之間的最小的平方差加和，可以抵抗異常值的出現
   從pts_obj和pts_img中選擇最合適的匹配點*/
    if(matches.size()>2)
      cv::solvePnPRansac( (cv::InputArray)pts_obj, (cv::InputArray)pts_img, (cv::InputArray)cameraMatrix,    H  /*cv::Mat()*/, rvec, tvec, false, 100,6.0, 100, inliers,cv::ITERATIVE);//可以計算出旋轉和平移的數組,最後一個值表示能找到的最好的匹配點的對數
 
/*對inliers進行輸出觀察輸出*/
cout<<"行："<<inliers.rows<<"     列："<<inliers.cols<<endl;
//correctIndex(inliers,  matches,t);
for(size_t r=0;r<inliers.rows;r++)
  {
    const ushort* rowPtr = inliers.ptr<ushort>(r);
       for(size_t c=0;c<inliers.cols;c++)
          {
            ushort pixel=rowPtr[c];
            cout<<pixel<<"  ";//scale 的範圍是10000
           //if((int)(uchar)rowPtr[c]!=0.0)
            //cout<<(int)(uchar)rowPtr[c]<<"    ";
          }
  }
    CvFont font; 
    double hscale = 0.51;
    double vscale = 0.5;
    int linewidth = 0.5;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX |CV_FONT_ITALIC,hscale,vscale,0,linewidth);

    CvScalar textColor =cvScalar(0,0,255);
    char num='a';
 
for (unsigned int j = 0; j < inliers.rows; ++j)//matches.size()
  { 
    const ushort* rowPtr = inliers.ptr<ushort>(j);
    int i=rowPtr[0];
    cout<<endl;
if(i!=t){
    cout<<"("<<matches[i].first.x<<","<<matches[i].first.y<<") ,  (";
    cout<<matches[i].second.x<<","<<matches[i].second.y<<")  ,  (" ;
    cout<<matches[i].first.x-matches[i].second.x<<","<<matches[i].first.y-matches[i].second.y<<")"<<endl;     
    drawPoint(img1,matches[i].first);
    drawPoint(img2,matches[i].second);
    const int & w = img1->width;//height;
    
    cvLine(img1,cvPoint(matches[i].first.x,matches[i].first.y),cvPoint(matches[i].second.x+w,matches[i].second.y), cvScalar(255,255,255),1);//ʹ\D3\C3w\B6Է\BD\CF\F2\BD\F8\D0\D0\C1\CB\C9\E8\D6\C3
    cvLine(img2,cvPoint(matches[i].first.x-w,matches[i].first.y),cvPoint(matches[i].second.x,matches[i].second.y), cvScalar(255,255,255),1);
    
    CvPoint textPos =cvPoint(matches[i].first.x,matches[i].first.y);
    cvPutText(img1, &num, textPos, &font,textColor);
    CvPoint textPos2 =cvPoint(matches[i].second.x,matches[i].second.y);
    cvPutText(img2, &num, textPos2, &font,textColor);
    num++;
 
}
}//for
 
      cout<<"inliers: "<<inliers.rows<<endl;
      cout<<"R="<<rvec<<endl;
      cout<<"t="<<tvec<<endl;

  cvNamedWindow("1", CV_WINDOW_AUTOSIZE );
  cvNamedWindow("2", CV_WINDOW_AUTOSIZE );
  cvShowImage("1", img1);
  cvShowImage("2",img2);
  cvWaitKey(0);

}

/*利用RANSAC函數，對特徵點的進行分類，分類後的結果作爲比較好的特徵點的集合進行接下來的旋轉和平移的計算*/
/*在有旋轉的情況下*/
void RANSACToMatch()
{

}







};

