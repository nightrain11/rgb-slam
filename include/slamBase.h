#pragma once
//各种头文件
//C++标准库
#include<fstream>
#include<iostream>
#include<vector>
#include<map>
using namespace std;
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//opencv
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>


//PCL
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/point_cloud.h>
#include<pcl-1.7/pcl/common/transforms.h>
#include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include<pcl-1.7/pcl/filters/voxel_grid.h>
//类定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT>PointCloud;

//相机内参
//b为双目相机的两摄像头的距离，f为焦距
struct CAMERA_INTRINSTIC_PARAMETERS
{
    double cx,cy,fx,fy,scale,b,f;
};

//函数接口
//image2PointCloud将rgb图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat& rgb,cv::Mat& depth,CAMERA_INTRINSTIC_PARAMETERS& camera);
PointCloud::Ptr image2PointCloud_bydisp( cv::Mat& rgb,CAMERA_INTRINSTIC_PARAMETERS& camera);
//input:3维点Point3f(u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point,CAMERA_INTRINSTIC_PARAMETERS& camera);
cv::Point3f point2dTo3d_bydisp(cv::Point3f&point,CAMERA_INTRINSTIC_PARAMETERS&camera);
//帧结构
struct FRAME

{
    int frameID;
    cv::Mat rgb,depth;//帧对应的彩色图与深度图
    cv::Mat desp;//特征描述子
    vector<cv::KeyPoint>kp;//关键点
    cv::Mat disp;
};
struct RESULT_OF_PNP
{
    cv::Mat rvec,tvec;
    int inliers;
};
void computeKeyPointsAndDesp(FRAME&fram,string detector,string descriptor);
RESULT_OF_PNP estimateMotion(FRAME&frame1,FRAME&frame2,CAMERA_INTRINSTIC_PARAMETERS&camera);
//读取参数类
class ParameterReader
{
public:
    ParameterReader( string filename="../parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSTIC_PARAMETERS& camera );
inline static CAMERA_INTRINSTIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSTIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}
