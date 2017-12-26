#include"slamBase.h"
PointCloud::Ptr image2PointCloud( cv::Mat& rgb,cv::Mat& depth,CAMERA_INTRINSTIC_PARAMETERS& camera)
{
    PointCloud::Ptr cloud( new PointCloud );
    for(int m=0;m<depth.rows;m++)
        for(int n=0;n<depth.cols;n++)
        {
            ushort d=depth.ptr<ushort>(m)[n];
            //d可能没有值，若如此，跳过此点
            if(d==0)
                continue;
            //d存在值，则向点云增加一个点
            PointT p;
            //d存在，则向点云增加一个点
            p.z=double(d)/camera.scale;
            p.x=(n-camera.cx)*p.z/camera.fx;
            p.y=-(m-camera.cy)*p.z/camera.fy;
            //从RGB图像获取它的颜色
            p.b=rgb.ptr<uchar>(m)[n*3];
            p.g=rgb.ptr<uchar>(m)[n*3+1];
            p.r=rgb.ptr<uchar>(m)[n*3+2];
            cloud->points.push_back(p);

        }
        cloud->height=1;
        cloud->width=cloud->points.size();
        cloud->is_dense=false;
        return cloud;
}
PointCloud::Ptr image2PointCloud_bydisp( cv::Mat& rgb,cv::Mat&disp,CAMERA_INTRINSTIC_PARAMETERS& camera)
{
    PointCloud::Ptr cloud( new PointCloud );
    for(int m=0;m<rgb.rows;m++)
        for(int n=0;n<rgb.cols;n++)
        {
            ushort d=disp.ptr<ushort>(m)[n];
            //d可能没有值，若如此，跳过此点
            if(d==0)
                continue;
            //d存在值，则向点云增加一个点
            PointT p;
            //d存在，则向点云增加一个点
            p.z=double(camera.b*camera.f)/d;
            p.x=(n-camera.cx)*p.z/camera.fx;
            p.y=-(m-camera.cy)*p.z/camera.fy;
            //从RGB图像获取它的颜色
            p.b=rgb.ptr<uchar>(m)[n*3];
            p.g=rgb.ptr<uchar>(m)[n*3+1];
            p.r=rgb.ptr<uchar>(m)[n*3+2];
            cloud->points.push_back(p);

        }
        cloud->height=1;
        cloud->width=cloud->points.size();
        cloud->is_dense=false;
        return cloud;
}
cv::Point3f point2dTo3d_bydisp(cv::Point3f&point,CAMERA_INTRINSTIC_PARAMETERS&camera)
{
    cv::Point3f p;
    p.z=double(camera.f*camera.b);//d为视差
    p.x=(point.x-camera.cx)*p.z/camera.fx;
    p.y=(point.y-camera.cy)*p.z/camera.fy;
    return p;
}
cv::Point3f point2dTo3d(cv::Point3f& point,CAMERA_INTRINSTIC_PARAMETERS& camera)
{
    cv::Point3f p;//3D点
    p.z=double(point.z)/camera.scale;
    p.x=(point.x-camera.cx)*p.z/camera.fx;
    p.y=(point.y-camera.cy)*p.z/camera.fy;
    return p;
}
void computeKeyPointsAndDesp(FRAME&frame,string detector,string descriptor)
{
    cv::Ptr<cv::FeatureDetector>_detector;
    cv::Ptr<cv::DescriptorExtractor>_descriptor;
    _detector=cv::FeatureDetector::create(detector.c_str());
    _descriptor=cv::DescriptorExtractor::create(descriptor.c_str());
    if(!_detector||!_descriptor)
    {
        cerr<<"Unknown detector or descriptor type !"<<detector<<","<<descriptor<<endl;
        return;
    }
    _detector->detect(frame.rgb,frame.kp);
    _descriptor->compute(frame.rgb,frame.kp,frame.desp);
    return;
}
//计算两个帧之间的运动
//输入：帧1，帧2
//输出：rvec和tvec
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSTIC_PARAMETERS& camera )
{
    static ParameterReader pd;
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );
   
    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if(matches[i].distance==0)
            continue;
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    
    cout<<"min dis = "<<minDis<<endl;
    if ( minDis < 10 ) 
        minDis = 10;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis )
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;

    if (goodMatches.size() <= 5) 
    {
        result.inliers = -1;
        return result;
    }
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}
RESULT_OF_PNP estimateMotion_bydisp( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSTIC_PARAMETERS& camera )
{
    static ParameterReader pd;
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );
   
    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    
    cout<<"min dis = "<<minDis<<endl;
    if ( minDis < 10 ) 
        minDis = 10;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis )
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;

    if (goodMatches.size() <= 5) 
    {
        result.inliers = -1;
        return result;
    }
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 相机内参
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.disp.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d_bydisp( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}
//该接口的作用是将旋转向量和平移向量转换为变换矩阵。
//输入：旋转矩阵rvec,平移向量tvec
//输出：Eigen::Isometry3d类型的变换矩阵
// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
//    cv::cv2eigen(R, r);
    for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
        r(i,j)=R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);
    return T;
}
// joinPointCloud 并对云图降采样
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSTIC_PARAMETERS& camera ) 
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );//设置滤波时创建的体素大小为？cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏  
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}
PointCloud::Ptr joinPointCloud_bydisp( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSTIC_PARAMETERS& camera ) 
{
    PointCloud::Ptr newCloud = image2PointCloud_bydisp( newFrame.rgb, newFrame.disp, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );//设置滤波时创建的体素大小为？cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏  
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}