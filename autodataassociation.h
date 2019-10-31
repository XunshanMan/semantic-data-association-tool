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


    void initialize(vector<Instance> &instances, Matrix3d& calib);

public:
    void initializeInstances(vector<Instance> &instances);
    vector<Association> process(VectorXd &poseTwc, MatrixXd &detMat);
    g2o::ellipsoid* generateEllipsoid(Instance& ins);

    cv::Mat drawProjection(cv::Mat& in);

private:
    vector<Association> associateProjEllipseAndDetections(vector<ProjEllipse>& projEllipses, MatrixXd &detMat);

private:
    vector<g2o::ellipsoid*> mvpEllipsoids;

    Matrix3d mmCalib;

    vector<ProjEllipse> mvProjEllipses;

};

#endif // AUTODATAASSOCIATION_H
