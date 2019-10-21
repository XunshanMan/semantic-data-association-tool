//
// Created by jhz on 19-8-22.
//

#include "utils/dataprocess_utils.h"
// 字符串读取和分割.
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <fstream>

#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace std;
using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;

Eigen::MatrixXd readDataFromFile(const char* fileName, bool dropFirstline){
    ifstream fin(fileName);
    string line;

    if(dropFirstline)
        getline(fin, line); // 扔掉第一行数据

    MatrixXd mat;
    int line_num = 0;
    while( getline(fin, line) )
    {
        vector<string> s;
        boost::split( s, line, boost::is_any_of( " \t," ), boost::token_compress_on );

        VectorXd lineVector(s.size());
        for (int i=0;i<int(s.size());i++)
            lineVector(i) = stod(s[i]);

        if(line_num == 0)
            mat.conservativeResize(1, s.size());
        else
            // vector to matrix.
            mat.conservativeResize(mat.rows()+1, mat.cols());

        mat.row(mat.rows()-1) = lineVector;

        line_num++;
    }
    fin.close();

    return mat;
}

// for multi objects
void selectKeyFrame(Eigen::MatrixXd &poses_mat_selected, std::vector<Eigen::MatrixXd> &detect_mat_selected,
                    Eigen::MatrixXd &poses_mat_key, std::vector<Eigen::MatrixXd> &detect_mat_key,
                    double deltaTheta, double deltaTrans){
    MatrixXd poses_key(0, poses_mat_selected.cols());
    std::vector<MatrixXd> detect_key;

    int rows = poses_mat_selected.rows();
    g2o::SE3Quat lastPose;
    for(int i=0;i<rows;i++)
    {
        Vector7d v = poses_mat_selected.row(i).tail(7);
        g2o::SE3Quat Twc(v);

        g2o::SE3Quat delta = lastPose.inverse() * Twc;
        Eigen::AngleAxisd rotation_vector(delta.rotation());
        if( delta.translation().norm() > deltaTrans && rotation_vector.angle() > deltaTheta ){
            // add one pose
            poses_key.conservativeResize(poses_key.rows()+1, poses_key.cols());
            poses_key.row(poses_key.rows()-1) =  poses_mat_selected.row(i);

            detect_key.push_back(detect_mat_selected[i]);

            lastPose = Twc;
        }

    }

    poses_mat_key = poses_key;
    detect_mat_key = detect_key;

}

void selectKeyFrame(Eigen::MatrixXd &poses_mat_selected, Eigen::MatrixXd &detect_mat_selected,
                    Eigen::MatrixXd &poses_mat_key, Eigen::MatrixXd &detect_mat_key,
                    double deltaTheta, double deltaTrans){
    MatrixXd poses_key(0, poses_mat_selected.cols());
    MatrixXd detect_key(0, detect_mat_selected.cols());

    int rows = poses_mat_selected.rows();
    g2o::SE3Quat lastPose;
    for(int i=0;i<rows;i++)
    {
        Vector7d v = poses_mat_selected.row(i).tail(7);
        g2o::SE3Quat Twc(v);

        g2o::SE3Quat delta = lastPose.inverse() * Twc;
        Eigen::AngleAxisd rotation_vector(delta.rotation());
        if( delta.translation().norm() > deltaTrans && rotation_vector.angle() > deltaTheta ){
            // add one pose
            poses_key.conservativeResize(poses_key.rows()+1, poses_key.cols());
            poses_key.row(poses_key.rows()-1) =  poses_mat_selected.row(i);

            detect_key.conservativeResize(detect_key.rows()+1, detect_key.cols());
            detect_key.row(detect_key.rows()-1) =  detect_mat_selected.row(i);

            lastPose = Twc;
        }

    }

    poses_mat_key = poses_key;
    detect_mat_key = detect_key;

}

void analyzeDetectionsAndStoreAsFrameAndID(std::vector<Eigen::MatrixXd> &detect_mat_key,
                                           std::vector<Eigen::MatrixXd> &bbox_mats)
{
    // 处理重复的检测. 暂时都按第一个.
    int num = detect_mat_key.size();

    std::set<int> label_sets;

    bbox_mats.resize(num);
    for(int i=0;i<num;i++)
    {
        MatrixXd detMat = detect_mat_key[i];
        bbox_mats[i].resize(0, detMat.cols());

        int totalDet = detMat.rows();
        for(int p=0;p<totalDet;p++)
        {
            VectorXd detVec = detMat.row(p);
            int objID = detVec(5);
            if( label_sets.find(objID) == label_sets.end() )   // 仅当没有时插入.
            {
                label_sets.insert(objID);
                bbox_mats[i].conservativeResize(bbox_mats[i].rows()+1, bbox_mats[i].cols());
                bbox_mats[i].row(bbox_mats[i].rows()-1) = detVec;
            }
        }
        label_sets.clear(); // 新的以帧 重新考虑检测.

    }

}

bool saveMatToFile(Eigen::MatrixXd &matIn, const char* fileName){
    ofstream fout;
    fout.open(fileName);

    int rows = matIn.rows();
    for(int i=0;i<rows;i++)
    {
        VectorXd v = matIn.row(i);
        int nums = v.rows();
        for( int m=0;m<nums;m++){
            fout << setprecision(12) << v(m);

            if( m== nums-1 )
                break;
            fout << " ";
        }

        fout << std::endl;
    }
    fout.close();

}

// 以下为InteriorNet数据集专场.
namespace InteriorNet{

bool loadPosesAndSingleObjectAnnotation(Eigen::MatrixXd &poses, Eigen::MatrixXd &detections){

    char fileName_cam[200] = "/home/jhz/liaoziwei/workspace/object-slam/ellipsoid-slam/core/test/debug-data/interiornet_poses.txt";
    char fileName_detect[200] = "/home/jhz/liaoziwei/workspace/object-slam/ellipsoid-slam/core/test/debug-data/interiornet_detections.txt";

    MatrixXd poses_mat = readDataFromFile(fileName_cam);
    MatrixXd detect_mat = readDataFromFile(fileName_detect);

    poses = poses_mat;
    detections = detect_mat;

    return true;
}

bool cameraPoseProcess(Eigen::MatrixXd &poses){
    // wxyz - > xyzw
    switchQuaternionOrder(poses);

    // 相机坐标系矫正
    calibrateCameraAxis(poses);

    // 将 Tcw转为 Twc
    // inverseEveryPose(poses);

    return true;
}

bool inverseEveryPose(Eigen::MatrixXd &poses){
    int rows = int(poses.rows());

    for(int i=0;i<rows;i++)
    {
        Vector7d v = poses.row(i).tail(7);
        g2o::SE3Quat Twc(v);
        g2o::SE3Quat Twc_trans = Twc.inverse();

        Vector7d v_done = Twc_trans.toVector();
        poses.row(i).tail(7) = v_done;
    }

    return true;
}

bool calibrateCameraAxis(Eigen::MatrixXd &poses){
    // 让其绕自身x轴旋转一个pi

    int rows = int(poses.rows());

    // 生成一个绕x轴转pi的变换.
    Vector6d v_trans;
    v_trans << 0,0,0,M_PI,0,0;
    g2o::SE3Quat tTrans;
    tTrans.fromXYZPRYVector(v_trans);

    for(int i=0;i<rows;i++)
    {
        Vector7d v = poses.row(i).tail(7);
        g2o::SE3Quat Twc(v);

        g2o::SE3Quat Twc_trans = Twc * tTrans;

        Vector7d v_done = Twc_trans.toVector();
        poses.row(i).tail(7) = v_done;

    }

    return true;

}

// 将 w x y z 的方式转为 x y z w 的方式.
bool switchQuaternionOrder(MatrixXd &poses)
{
    // timestamp x y z qw qx qy qz
    MatrixXd qRows = poses.block(0, 4, poses.rows(), 4);

    MatrixXd qRowsOrdered(qRows.rows(), qRows.cols());
    qRowsOrdered.col(0) = qRows.col(1);
    qRowsOrdered.col(1) = qRows.col(2);
    qRowsOrdered.col(2) = qRows.col(3);
    qRowsOrdered.col(3) = qRows.col(0);  // qw

    poses.block(0, 4, poses.rows(), 4) = qRowsOrdered;

    return true;

}

cv::Mat processDepth(cv::Mat& depth, double fx, double cx, double cy){

    cv::Mat output(depth.rows, depth.cols, CV_16UC1);

    for( int y = 0; y< output.rows; y++)
        for( int x = 0; x< output.cols; x++)
        {
            ushort d = depth.at<ushort>(y,x);
            double realz = calculateRealZ(fx, double(d), (x - cx), (y - cy));
            output.at<ushort>(y, x)  = ushort(realz);

        }

    return output;

}

double calculateRealZ(double f, double d, double xi, double yi){
    return f*sqrt(d*d/(xi*xi+f*f+yi*yi));
}




}