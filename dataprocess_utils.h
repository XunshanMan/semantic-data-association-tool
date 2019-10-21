//
// Created by jhz on 19-8-22.
//

#ifndef ELLIPSOIDSLAM_DATAPROCESS_UTILS_H
#define ELLIPSOIDSLAM_DATAPROCESS_UTILS_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

// 从txt文件中读取并存储为Eigen数组格式
Eigen::MatrixXd readDataFromFile(const char* fileName, bool dropFirstline = false);
void selectKeyFrame(Eigen::MatrixXd &poses_mat_selected, Eigen::MatrixXd &detect_mat_selected,
                    Eigen::MatrixXd &poses_mat_key, Eigen::MatrixXd &detect_mat_key,
                    double deltaTheta, double deltaTrans = 0);
// Multi objects version
void selectKeyFrame(Eigen::MatrixXd &poses_mat_selected, std::vector<Eigen::MatrixXd> &detect_mat_selected,
                    Eigen::MatrixXd &poses_mat_key, std::vector<Eigen::MatrixXd> &detect_mat_key,
                    double deltaTheta, double deltaTrans = 0);
// 从检测框矩阵中提取并存储为新结构
void analyzeDetectionsAndStoreAsFrameAndID(std::vector<Eigen::MatrixXd> &detect_mat_key,
                                           std::vector<Eigen::MatrixXd> &bbox_mat);

// 将Mat数据存放
bool saveMatToFile(Eigen::MatrixXd &matIn, const char* fileName);

// 为数据集使用.
namespace InteriorNet {
    // 一键读取所需的所有数据.
    bool loadPosesAndSingleObjectAnnotation(Eigen::MatrixXd &poses, Eigen::MatrixXd &detections);

    bool cameraPoseProcess(Eigen::MatrixXd &poses);
    bool switchQuaternionOrder(Eigen::MatrixXd &poses);
    bool calibrateCameraAxis(Eigen::MatrixXd &poses);
    bool inverseEveryPose(Eigen::MatrixXd &poses);

    // InteriorNet 的depth数据需要做预处理.
    cv::Mat processDepth(cv::Mat& depth, double fx, double cx, double cy);

    double calculateRealZ(double f, double d, double xi, double yi);

}

#endif //ELLIPSOIDSLAM_DATAPROCESS_UTILS_H
