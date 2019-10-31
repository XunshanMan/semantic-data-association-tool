#include "autodataassociation.h"

#include "include/core/g2o_object.h"
#include "instance.h"

#include <Eigen/Core>

#include "DataStruct.h"

#include <algorithm>

using namespace Eigen;
using namespace std;

bool compare_func(pair<int,double>& p1, pair<int,double>& p2)
{
    return p1.second < p2.second;
}

void AutoDataAssociation::initialize(vector<Instance> &instances, Matrix3d& calib)
{

    mmCalib = calib;
    initializeInstances(instances);

}

void AutoDataAssociation::initializeInstances(vector<Instance> &instances)
{
    mvpEllipsoids.clear();
    int num = instances.size();
    for(int i=0;i<num;i++){
        auto ins = instances[i];
        g2o::ellipsoid* e = generateEllipsoid(ins);
        mvpEllipsoids.push_back(e);
    }

    return;

}

// 输入: 当前帧gt,
vector<Association> AutoDataAssociation::process(VectorXd &poseTwc, MatrixXd &detMat){
    int objNum = mvpEllipsoids.size();

    vector<ProjEllipse> projEllipses;
    // 遍历 instances
    for(int i=0;i<objNum;i++)
    {
        g2o::ellipsoid* e = mvpEllipsoids[i];

        g2o::SE3Quat Twc(poseTwc.tail(7));  // xyz q4
        Vector4d bbox = e->getBoundingBoxFromProjection(Twc.inverse(), mmCalib);

        // 定义一个投影体, 方便彼此进行数据关联.
        ProjEllipse proje;
        proje.label = e->miLabel;
        proje.bbox = bbox;
        proje.id = i;

        projEllipses.push_back(proje);
    }

    // 开始做关联
    vector<Association> associations = associateProjEllipseAndDetections(projEllipses, detMat);

    return associations;


}

g2o::ellipsoid* AutoDataAssociation::generateEllipsoid(Instance& ins){
    g2o::ellipsoid* e = new g2o::ellipsoid;
    Eigen::Matrix<double, 9, 1> v;
    for(int n=0;n<9;n++)
        v(n) = ins.param[n];
    e->fromMinimalVector(v);

    // 设置 label id.
    e->miLabel = ins.label;

    return e;
}

vector<Association> AutoDataAssociation::associateProjEllipseAndDetections(vector<ProjEllipse>& projEllipses, MatrixXd &detMat)
{
    double THRESH_DIS = 150; // 中心 x y差距和不超过多少像素
    // -------
    vector<Association> associations;
    vector<ProjEllipse> projEllipsesSelected; // 存储那些被选召的椭圆

    // 针对每个ob det 从候选椭球挑一个
    //  -> 依据: label一致(?错误检测, 很少的情况不考虑), 挑选距离最近, 而且距离满足阈值.
    int detNum = detMat.rows();

    vector<ProjEllipse> projEllipsesLeft = projEllipses;
    int objNum = projEllipsesLeft.size();
    vector<bool> projsChosen(objNum, false);
    for( int i=0; i< detNum; i++ )
    {
        Association asso;

        VectorXd det = detMat.row(i);
        // 挑选出所有label一致的.
        int label = det(DET_TABLE_LABEL);
        double x1 = det(DET_TABLE_X1);
        double y1 = det(DET_TABLE_Y1);
        double x2 = det(DET_TABLE_X2);
        double y2 = det(DET_TABLE_Y2);

        if(objNum > 0){
            vector<pair<int,double>> associationDistances;
            for(int n=0;n<objNum;n++)
            {
                if(projsChosen[n]) continue;        // 已经选中的不再考虑
                ProjEllipse proj = projEllipsesLeft[n];
                // if(proj.label!=label) continue;

                // 计算矩形框中间差值
                double dis = std::abs(x1+x2-proj.bbox(0)-proj.bbox(2))/2
                        + std::abs(y1+y2-proj.bbox(1)-proj.bbox(3))/2;
                associationDistances.push_back(make_pair(n, dis));
            }

            if(associationDistances.size() > 0){
                // 排序取最小的一个.
                sort(associationDistances.begin(), associationDistances.end(), compare_func);

                double mini_dis = associationDistances[0].second;
                int best_instance_id = associationDistances[0].first;
                if(mini_dis < THRESH_DIS )
                {
                    // ok
                    Association asso;
                    asso.instanceID = best_instance_id;
                    asso.detID = i;
                    asso.prob = 1;

                    associations.push_back(asso);

                    projsChosen[best_instance_id] = true;
                    projEllipsesSelected.push_back(projEllipses[best_instance_id]);

                }
            }
        }

    }
    // 依次循环得到关联结果.
    mvProjEllipses.clear();
    mvProjEllipses = projEllipsesSelected;

    return associations;
}

cv::Mat AutoDataAssociation::drawProjection(cv::Mat &in)
{
    cv::Mat out = in.clone();
    int num = mvProjEllipses.size();
    for(auto proj : mvProjEllipses )
    {
        Vector4d rect = proj.bbox;
        cv::rectangle(out, cv::Rect(cv::Point(rect[0],rect[1]),cv::Point(rect[2],rect[3])), cv::Scalar(0,0,255), 2);
    }

    return out.clone();
}
