#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <string>
#include "core/Viewer.h"
#include "core/Geometry.h"

#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#include "instance.h"

using namespace std;

class visualizer{

public:

void startVisualizationThread(string &path_setting);
void addBackgroundPoint(string &path_pcd);

void refreshInstances(vector<Instance> &instances);
void initializeInstances(vector<Instance> &instances);

bool isInitialized();

void addTrajectory(Eigen::MatrixXd &poseMat);
void clearTrajectory();

visualizer();

private:
    QuadricSLAM::System* mpSystem;
    QuadricSLAM::Map* mpMap;

    bool mbInitialized;

private:
    QuadricSLAM::PointCloud pclToQuadricPointCloud(PointCloud::Ptr &pCloud);
    g2o::ellipsoid* generateEllipsoid(Instance& ins);
};

#endif // VISUALIZE_H
