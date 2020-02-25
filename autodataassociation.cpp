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

void AutoDataAssociation::initialize(vector<Instance> &instances, Matrix3d& calib, int rows, int cols)
{

    mmCalib = calib;
    mRows = rows;
    mCols = cols;
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

// 若是边界则输出 True
bool AutoDataAssociation::calibrateMeasurement(Vector4d &measure , int rows, int cols){
    int config_boarder = 10;    // 20个像素边界
    int config_size = 100;

    // 添加对大小的过滤
    int x_length = measure[2] - measure[0];
    int y_length = measure[3] - measure[1];
    if( x_length < config_size || y_length < config_size ){
        std::cout << " [small detection " << config_size << "] invalid. " << std::endl;
        return true;
    }

    Vector4d measure_calibrated(-1,-1,-1,-1);
    Vector4d measure_uncalibrated(-1,-1,-1,-1);

    int correct_num = 0;
    if(  measure[0]>config_boarder && measure[0]<cols-1-config_boarder )
    {
        measure_calibrated[0] = measure[0];
        correct_num++;
    }
    if(  measure[2]>config_boarder && measure[2]<cols-1-config_boarder )
    {
        measure_calibrated[2] = measure[2];
        correct_num++;
    }
    if(  measure[1]>config_boarder && measure[1]<rows-1-config_boarder )
    {
        measure_calibrated[1] = measure[1];
        correct_num++;
    }
    if(  measure[3]>config_boarder && measure[3]<rows-1-config_boarder )
    {
        measure_calibrated[3] = measure[3];
        correct_num++;
    }

    // DEBUG: 一旦有一个在边缘，我们直接舍弃掉这个边的误差！
    // if( correct_num == 4)
    //     measure = measure_calibrated;
    // else
    //     measure = measure_uncalibrated;

    measure = measure_calibrated;

    if( correct_num != 4)
        return true;
    else
        return false;

}

void AutoDataAssociation::GetValidDetMat(MatrixXd &detMat, MatrixXd &detMatValid, std::vector<int> &originPos)
{
    int num = detMat.rows();

    MatrixXd matValid; matValid.resize(0, detMat.cols());
    std::vector<int> posVec;
    for( int i=0; i<num; i++)
    {
        VectorXd detVec = detMat.row(i);

        Vector4d measure; measure << detVec[1], detVec[2], detVec[3], detVec[4];
        bool is_border = calibrateMeasurement(measure, mRows, mCols);

        bool c1 = !is_border;  // 不能是边界, 小物体

        if( c1 )
        {
            matValid.conservativeResize(matValid.rows()+1, matValid.cols());
            matValid.row(matValid.rows()-1) =  detVec;
            posVec.push_back(i);
        }
    }

    detMatValid = matValid;
    originPos = posVec;
}

// 输入: 当前帧gt,
vector<Association> AutoDataAssociation::process(VectorXd &poseTwc, MatrixXd &detMat, bool add_condition){

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

    // 依次循环得到关联结果.
    mvProjEllipses.clear();
    mvProjEllipses = projEllipses;

    // 开始做关联
    vector<Association> associations;
    if(!add_condition)
        associations = associateProjEllipseAndDetections(projEllipses, detMat);
    else
    {
        // 筛选满足条件的 det组成新的 mat, 同时用一个 vector 记录它们的原始位置
        std::vector<int> originPos;
        MatrixXd detMatValid;
        GetValidDetMat(detMat, detMatValid, originPos);

        vector<Association> associations_temp = associateProjEllipseAndDetections(projEllipses, detMatValid);

        associations.clear();
        for( auto as : associations_temp )
        {
            as.detID = originPos[as.detID];     // 切换回原来的id.
            associations.push_back(as);
        }

    }
    

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
    double THRESH_DIS = 100; // 中心 x y差距和不超过多少像素
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


    return associations;
}

cv::Mat AutoDataAssociation::drawVisualization(cv::Mat &oriMat, MatrixXd &mmDetMat, vector<Association> &associations)
{
    cv::Mat mat = drawProjection(oriMat);
    // cv->
    mat = drawBboxMat(mat, mmDetMat);

    // draw association
    for( auto asso : associations )
    {
        if ( asso.instanceID == -1 ) continue;
        VectorXd detVec = mmDetMat.row(asso.detID);
        cv::Point det_point(detVec[1], detVec[2]);
        
        auto proj = mvProjEllipses[asso.instanceID];
        cv::Point proj_point(proj.bbox[0],proj.bbox[1]);

        cv::line(mat, det_point, proj_point, cv::Scalar(0,255,0), 2);
    }
    return mat;
}

cv::Mat AutoDataAssociation::drawProjection(cv::Mat &in)
{
    cv::Mat out = in.clone();
    int num = mvProjEllipses.size();

    int ins_id = 0;
    for(auto proj : mvProjEllipses )
    {
        Vector4d rect = proj.bbox;
        cv::rectangle(out, cv::Rect(cv::Point(rect[0],rect[1]),cv::Point(rect[2],rect[3])), cv::Scalar(0,0,255), 2);
        cv::putText(out, to_string(ins_id++), cv::Point(rect(2), rect(3)), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255));

    }

    return out.clone();
}

cv::Mat AutoDataAssociation::drawBboxMat(cv::Mat &im, Eigen::MatrixXd &mat_det)
{
    cv::Mat mImage = im.clone();
    for( int r = 0; r<mat_det.rows(); r++)
    {
        VectorXd vDet = mat_det.row(r);
        Vector4d measure; measure << vDet[1], vDet[2], vDet[3], vDet[4];
        bool is_border = calibrateMeasurement(measure, mRows, mCols);

        if( is_border ) continue;

        int labelId = int(vDet(5));
        int colorId = labelId % 255;

        //cout << "vDet " << vDet << endl;
        // 如果检测有效. id号能匹配上

        cv::Rect rec(cv::Point(vDet(1), vDet(2)), cv::Point(vDet(3), vDet(4)));
        cv::rectangle(mImage, rec, cv::Scalar(255,0,0), 2);
        cv::putText(mImage, to_string(labelId), cv::Point(vDet(1), vDet(2)), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255,0,0));
        cv::putText(mImage, to_string(r), cv::Point(vDet(3), vDet(4)), CV_FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0,255,0));

    }

    return mImage.clone();    
}