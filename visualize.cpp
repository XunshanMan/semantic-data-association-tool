// 本文件处理与Ellipsoid Core之间的交互

#include "visualize.h"

#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>

#include <vector>
#include "instance.h"

#include <Eigen/Core>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

void visualizer::startVisualizationThread(string &path_setting){
    // 尝试使用 Viewer可视化这一切.
    mpSystem = new QuadricSLAM::System(path_setting, true);
    mpMap = mpSystem->getMap();

    mbInitialized = true;
}


void visualizer::addBackgroundPoint(string &path_pcd){
    // 读取pcd
    std::cout << "Load pcd..." << std::endl;
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(path_pcd.c_str(), *cloud) == -1)
    {
        std::cout << "Cloudn't read file!" << std::endl;
        return;
    }
    std::cout << "ok." << std::endl;
    // pcd 转换为QuadricSLAM下的 PointCloud
    auto cloudQuadric = new QuadricSLAM::PointCloud(pclToQuadricPointCloud(cloud));
    mpMap->addPointCloud(cloudQuadric);   // 测试局部初始化结果.

}

void visualizer::refreshInstances(vector<Instance> &instances){
    vector<g2o::ellipsoid*> pEllipsoids = mpMap->GetAllEllipsoids();
    if( pEllipsoids.size() > 0)
    {
        // 如果需要添加新的instances
        if(instances.size() > pEllipsoids.size())
        {
            vector<Instance> newInstances;
            newInstances.assign(instances.begin()+pEllipsoids.size(), instances.end());
            initializeInstances(newInstances);
        }

        // 更新
        int num = pEllipsoids.size();
        for( int i=0;i<num;i++)
        {
            g2o::ellipsoid* e = pEllipsoids[i];
            Eigen::Matrix<double, 9, 1> v;
            for(int n=0;n<9;n++)
                v(n) = instances[i].param[n];
            e->fromMinimalVector(v);
        }
    }
    else
    {
        initializeInstances(instances);
    }
}

// 开辟内存空间, 之后就不开辟了。
void visualizer::initializeInstances(vector<Instance> &instances){

    int num = instances.size();
    for(int i=0;i<num;i++)
    {
        g2o::ellipsoid* pEllipsoid = generateEllipsoid(instances[i]);
        mpMap->addEllipsoid(pEllipsoid);
    }
}

void showObject(){
    // 更新地图中的物体表示

//    clearMapObjects();

//    generateEllipsoidFromInstance();

//    addEllipsoidToMap();

//    // // 地图中添加可视化物体
//    Eigen::Vector3d red_color(1.0,0,0);
//    eOptimized.setColor(red_color); // red
//    mpMap->addEllipsoid(&eInit);
//    mpMap->addEllipsoid(&eOptimized);

//    // 添加点云
//    // TODO

//    // 添加可视化的三维点云.
//    //PointCloudXYZ::Ptr point_cloud = QuadricSLAM::PointGroup::loadPointsToPointCloud(pointsMatFiltered);
//    QuadricSLAM::PointCloud orbPoints = QuadricSLAM::loadPointsToPointVector(pointsMatFiltered);
//    mpSystem->getMap()->addPointCloud(&orbPoints);      // 注意orb点不带尺度，无法融合!

}

QuadricSLAM::PointCloud visualizer::pclToQuadricPointCloud(PointCloud::Ptr &pCloud)
{
    QuadricSLAM::PointCloud cloud;
    int num = pCloud->points.size();
    for(int i=0;i<num;i++){
        QuadricSLAM::PointXYZRGB p;
        PointT pT = pCloud->points[i];
        p.r = pT.r;
        p.g = pT.g;
        p.b = pT.b;

        p.x = pT.x;
        p.y = pT.y;
        p.z = pT.z;
        cloud.push_back(p);
    }

    return cloud;
}

g2o::ellipsoid* visualizer::generateEllipsoid(Instance& ins){
    g2o::ellipsoid* e = new g2o::ellipsoid;
    Eigen::Matrix<double, 9, 1> v;
    for(int n=0;n<9;n++)
        v(n) = ins.param[n];
    e->fromMinimalVector(v);

    // 设置 label id.
    e->miLabel = ins.label;

    return e;
}

bool visualizer::isInitialized(){
    return mbInitialized;
}

visualizer::visualizer():mbInitialized(false)
{

}

void visualizer::addTrajectory(Eigen::MatrixXd &poseMat){
    std::cout << "Visualize trajectory... " << std::endl;
    for(int i=0; i<poseMat.rows(); i++){
        g2o::SE3Quat* pTwc = new g2o::SE3Quat(poseMat.row(i).tail(7));
        mpMap->addCameraStateToTrajectory(pTwc);
    }
}

void visualizer::clearTrajectory()
{
//    std::vector<g2o::SE3Quat*> states = mpMap->getCameraStateTrajectory();
//    // release
//    for(auto p : states)
//        delete p;

//    mpMap->

}
