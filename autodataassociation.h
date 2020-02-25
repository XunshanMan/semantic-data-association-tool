#ifndef AUTODATAASSOCIATION_H
#define AUTODATAASSOCIATION_H

#include "instance.h"
#include <vector>
#include "include/core/g2o_object.h"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

class ProjEllipse
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id; // instance ID
    int label;
    Vector4d bbox;
};

struct Association
{
    double prob;
    int instanceID;
    int detID;
};

class AutoDataAssociation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void initialize(vector<Instance> &instances, Matrix3d& calib, int rows, int cols);
    
    cv::Mat drawBboxMat(cv::Mat &im, Eigen::MatrixXd &mat_det);

public:
    void initializeInstances(vector<Instance> &instances);
    vector<Association> process(VectorXd &poseTwc, MatrixXd &detMat, bool add_condition);   // add_condition: 仅仅挑选符合条件的参与
    g2o::ellipsoid* generateEllipsoid(Instance& ins);

    cv::Mat drawProjection(cv::Mat& in);

    cv::Mat drawVisualization(cv::Mat &oriMat, MatrixXd &mmDetMat, vector<Association> &associations);
private:
    vector<Association> associateProjEllipseAndDetections(vector<ProjEllipse>& projEllipses, MatrixXd &detMat);

    bool calibrateMeasurement(Vector4d &measure , int rows, int cols);
    void GetValidDetMat(MatrixXd &detMat, MatrixXd &detMatValid, std::vector<int> &originPos);


private:
    vector<g2o::ellipsoid*> mvpEllipsoids;

    Matrix3d mmCalib;
    int mRows;
    int mCols;

    vector<ProjEllipse> mvProjEllipses;

};

#endif // AUTODATAASSOCIATION_H
