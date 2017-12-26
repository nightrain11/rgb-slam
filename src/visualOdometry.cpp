#include<iostream>
#include<fstream>
#include<sstream>
using namespace std;
#include"slamBase.h"
FRAME readFrame(int index,ParameterReader&pd);
//度量运动的大小
double normofTransform(cv::Mat rvec,cv::Mat tvec);
int main(int argc,char**argv)
{
    ParameterReader pd;
    int startIndex=atoi(pd.getData("start_index").c_str());
    int endIndex=atoi(pd.getData("end_index").c_str());
    //初始化
    cout<<"initializing"<<endl;
    int currIndex=startIndex;
    FRAME lastFrame=readFrame(currIndex,pd);
    string detector=pd.getData("detector");
    string descriptor=pd.getData("descriptor");
    CAMERA_INTRINSTIC_PARAMETERS camera=getDefaultCamera();
    computeKeyPointsAndDesp(lastFrame,detector,descriptor);
    PointCloud::Ptr cloud=image2PointCloud(lastFrame.rgb,lastFrame.depth,camera);
    pcl::visualization::CloudViewer viewer("viewer");
    //是否显示亮点
    bool visualize=pd.getData("visualization_pointcloud")==string("yes");
    int min_inliers=atoi(pd.getData("min_inliers").c_str());
    double max_norm=atof(pd.getData("max_norm").c_str());
    for(currIndex=startIndex+1;currIndex<endIndex;currIndex++)
    {
        cout<<"Reading files"<<currIndex<<endl;
        FRAME currFrame=readFrame(currIndex,pd);
        computeKeyPointsAndDesp(currFrame,detector,descriptor);
        //比较currFrame和lastFrame
        RESULT_OF_PNP result =estimateMotion(lastFrame,currFrame,camera);
        if(result.inliers<min_inliers)//inliers不够
            continue;
//度量运动大小的值：∥Δt∥+min(2π−∥r∥,∥r∥)
        double norm=normofTransform(result.rvec,result.tvec);
        cout<<"norm= "<<norm<<endl;
        if(norm>=max_norm)
            continue;
        //欧式变换
        Eigen::Isometry3d T=cvMat2Eigen(result.rvec,result.tvec);
        cout<<"T="<<T.matrix()<<endl;
        cloud=joinPointCloud(cloud,currFrame,T,camera);
        if(visualize==true)
            viewer.showCloud(cloud);
        lastFrame=currFrame;

    }
pcl::io::savePCDFile("../data/result.pcd",*cloud);
return 0;

}
FRAME readFrame(int index,ParameterReader&pd)
{
    FRAME f;
    string rgbDir=pd.getData("rgb_dir");
    string depthDir=pd.getData("depth_dir");
    string rgbExt=pd.getData("rgb_extension");
    string depthExt=pd.getData("depth_extension");
    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb=cv::imread(filename);
    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;
    f.depth=cv::imread(filename,-1);
    return f;
}
double normofTransform( cv::Mat rvec, cv::Mat tvec )
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}