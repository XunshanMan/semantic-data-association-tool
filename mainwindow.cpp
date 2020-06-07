#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utility.h"

#include <vector>
#include <string>

#include <iostream>

// 快捷键
#include <QShortcut>
#include <QString>

#include "dataprocess_utils.h"
#include <Eigen/Core>

#include "instance.h"

// 可视化插件
#include "visualize.h"

#include "autodataassociation.h"
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 初始化图像显示
    frame = new QFrame(ui->label);
//    frame->resize(640,480);
    // frame->setGeometry(0,0,640,480);
    frame->setGeometry(0,0,960,540);
    pixmap = new QPixmap();
    palette = new QPalette();

    // 注册快捷键
    QShortcut *shortcut_left = new QShortcut(QKeySequence(Qt::Key_Left), this);
    QObject::connect(shortcut_left, SIGNAL(activated()), this, SLOT(on_pushButton_left_clicked()));

    QShortcut *shortcut_right = new QShortcut(QKeySequence(Qt::Key_Right), this);
    QObject::connect(shortcut_right, SIGNAL(activated()), this, SLOT(on_pushButton_right_clicked()));

    // 初始化table性能
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableWidget_instance->setSelectionBehavior(QAbstractItemView::SelectRows);

    mbInstanceTableInitiated = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::refreshText(){

    QString strShow = QString::number(miCurrentID) + QString(" / ") + QString::number(miTotalNum);
    ui->label_num->setText( strShow );
}

void MainWindow::on_pushButton_clicked()
{
    QString strPath = ui->lineEdit_detection_dir->text();
    GetFileNames(strPath.toStdString(), mDetectionFiles );
    sortFileNames(mDetectionFiles, mDetectionFiles);

    miTotalNum = mDetectionFiles.size();

    // 更新图像位置
    msImagePath = ui->lineEdit_imagePath->text().toStdString();
    // 更新检测位置
    msDetectPath = ui->lineEdit_detection_dir->text().toStdString();
    // 更新标签.
    msLabelConfigPath = ui->lineEdit_labelConfig->text().toStdString();
    readLabelConfig(msLabelConfigPath);
    // 更新实例.
    msInstancesPath = ui->lineEdit_instancePath->text().toStdString();
    readInstances(msInstancesPath);
    refreshInstanceTable();

    // 读取第0张图.
    miCurrentID = 0;
    dealWithPic(miCurrentID);

    // 加载自动数据关联.
    Matrix3d calib;
//    calib << 517.3, 0, 318.6,
//                0, 516.5, 255.3,
//                0, 0, 1;
    // For TUM fr1
    // calib << 520.9, 0, 325.1,
    //     0, 521.0, 249.7,
    //     0, 0, 1;
    // For TUM fr2

    // For kinect2
    // Camera.fx: 540.68603515625
    // Camera.fy: 540.68603515625
    // Camera.cx: 479.75
    // Camera.cy: 269.75
    calib << 540.68603515625, 0, 479.75,
        0, 540.68603515625, 269.75,
        0, 0, 1;
    
    int rows = 540;
    int cols = 960;

    std::cout << "Load calibration file : " << std::endl << calib << std::endl;

    mAssociater.initialize(mvInstances, calib, rows, cols);

    // 初始化数据集工具
    msDatasetDir = ui->lineEdit_datasetDir->text().toStdString();
    mLoader.loadDataset(msDatasetDir);

}

void MainWindow::dealWithPic(int id){
    cout << "Deal with " << id << endl;

    // 读取图像切换.
    string detFullPath = mDetectionFiles[id];
    string bareFileName = splitFileName(detFullPath, true);
    string imageFullPath = msImagePath + bareFileName + ".png";

    // 处理opencv相关图像.
    std::cout << "bareFileName: " << bareFileName << std::endl;
    std::cout << "image path: " << imageFullPath << std::endl;
    (*pixmap) = QPixmap(imageFullPath.c_str());

    palette->setBrush(frame->backgroundRole(),QBrush(*pixmap));
    frame->setPalette(*palette);
    frame->setAutoFillBackground(true);
    //frame->setMask(pixmap->mask());  //可以将图片中透明部分显示为透明的
    frame->show();

    refreshText();

    // 读取检测
    readDetection(id);

    // 进行自动数据关联
    if(ui->checkBox->isChecked())
        automaticAssociation();

}

// 上一个
void MainWindow::on_pushButton_left_clicked()
{
    if(miCurrentID>0){
        saveCurrentDetection();        // 保存当前情况

        miCurrentID--;
        dealWithPic(miCurrentID);
    }
}

// 下一个
void MainWindow::on_pushButton_right_clicked()
{
    if(miCurrentID<miTotalNum-1){
        saveCurrentDetection();        // 保存当前情况

        miCurrentID++;
        dealWithPic(miCurrentID);
    }
}

void MainWindow::readDetection(int id){
    string detFileName = mDetectionFiles[id];

    // 读取det数据.
    MatrixXd detMat = readDataFromFile(detFileName.c_str());
    std::cout << detMat << std::endl;

    // 绘制det结果到控件中. 表格?
    refreshTable(detMat);

    // 保存当前detMat
    mmDetMat = detMat;

}

void MainWindow::refreshTable(MatrixXd &mat){
    int rows = mat.rows();

    QTableWidget* pTable = ui->tableWidget;
    pTable->clearContents();
    pTable->setRowCount(rows);
    for( int i=0;i<rows;i++){
        VectorXd v = mat.row(i);

        for( int p=0;p<mat.cols();p++)
        {
            double data = v(p);
            //string strData = to_string(data);
            //std::cout << i << ", " << p << ": " << strData << std::endl;
            pTable->setItem(i, p, new QTableWidgetItem(QString::number(data)));

            // 处理 label 栏
            if (p==5)
            {
                QString str;
                str = QString::number(data) + QString(" / ") + QString::fromStdString(mvLabelIDToString[int(data)]);
                pTable->setItem(i, p, new QTableWidgetItem(str));
            }
        }

    }

}

void MainWindow::readLabelConfig(string &path){
    mvLabelIDToString = readStringFromFile(path.c_str());
}

void MainWindow::readInstances(string &path){
    MatrixXd instanceMat = readDataFromFile(path.c_str(), false);        // 剔除首行
    int num = instanceMat.rows();

    mvInstances.clear();
    mvInstances.resize(num);
    for( int i=0; i<num; i++)
    {
        VectorXd v = instanceMat.row(i);

        Instance obj;
        obj.id = int(v(0));
        for(int n=0;n<9;n++)
            obj.param[n] = double(v(n+1));

        if(v.size()==11)
            obj.label = int(v(10));
        else obj.label = 0;

        mvInstances[i] = obj;
    }

    mvInstancesWithRot = mvInstances;


}

void MainWindow::refreshInstanceTable(){
    int num = mvInstances.size();
    std::cout << "instance num : " << num << std::endl;
    QTableWidget* pTable = ui->tableWidget_instance3D;
    pTable->clearContents();
    pTable->setRowCount(num);
    for( int i=0; i<num;i++ )
    {
        Instance* pObj = &mvInstances[i];
        pTable->setItem(i, INS_TABLE_ID, new QTableWidgetItem(QString::number(pObj->id)));

        for( int n=0; n<9; n++)
        {
            QTableWidgetItem* pItem = new QTableWidgetItem(QString::number(pObj->param[n]));
            pTable->setItem(i, n+1, pItem);
        }

        // 将地平面额外旋转置0
        pTable->setItem(i, INS_TABLE_GPROT, new QTableWidgetItem(QString::number(0)));

        int label = pObj->label;
        QString str;
        str = QString::number(label) + QString(" / ") + QString::fromStdString(mvLabelIDToString[label]);
        pTable->setItem(i, INS_TABLE_LABEL, new QTableWidgetItem(str));

    }

    mbInstanceTableInitiated = true;
}





void MainWindow::on_tableWidget_itemSelectionChanged()
{
    int currentRow = ui->tableWidget->currentRow();

    QTableWidgetItem* pItem = ui->tableWidget->item(currentRow, DET_TABLE_INSTANCE);

    if(pItem!=NULL){
        QString text = pItem->text();
        std::cout << "text: " << text.toStdString() << std::endl;

        int instance = text.toInt();
        std::cout << "instance: " << instance << std::endl;
        if( instance > 0)
            ui->tableWidget_instance->setCurrentCell(instance, 0);
    }
    else
    {
        // 尚未安排Instance
        //ui->tableWidget_instance->
    }
}

void MainWindow::saveCurrentDetection(){
    if(mDetectionFiles.size() < 1) return;
    string currentPath = mDetectionFiles[miCurrentID];

    // 主要处理detMat的最后一列.
    int rows = ui->tableWidget->rowCount();
    int cols = ui->tableWidget->columnCount();

    // 第一种情况，尚未做标注， detMat的size比表格小.
    if( cols > mmDetMat.cols() )
        mmDetMat.conservativeResize(mmDetMat.rows(), cols);

    // 修改最后一列
    for( int i =0; i<rows; i++)
    {
        QTableWidgetItem* pItem = ui->tableWidget->item(i, DET_TABLE_INSTANCE);

        if(pItem!=NULL)
        {
            mmDetMat(i,DET_TABLE_INSTANCE) = pItem->text().toInt();
        }
        else
        {
            mmDetMat(i,DET_TABLE_INSTANCE) = -1;
        }

    }

    // 保存mmDetMat
    saveMatToFile(mmDetMat, currentPath.c_str());
}

void MainWindow::on_pushButton_3_clicked()
{
    saveCurrentDetection();
}

void MainWindow::on_pushButton_4_clicked()
{
    saveCurrentDetection();

    // Save all files.
    saveCurrentInstances();

    this->destroy();
}

void MainWindow::on_tableWidget_instance_itemSelectionChanged()
{
    // 若此时 det table 被选中，则更换其instance
    int currentRow = ui->tableWidget->currentRow();

    int currentInstance = ui->tableWidget_instance->currentRow();
    if( currentRow > -1 )
    {
        QTableWidgetItem* item = ui->tableWidget->item(currentRow, DET_TABLE_INSTANCE);
        if(item == NULL)
        {
            ui->tableWidget->setItem(currentRow, DET_TABLE_INSTANCE, new QTableWidgetItem(QString::number(currentInstance)));
        }
        else
            item->setText(QString::number(currentInstance));
    }
}

void MainWindow::on_pushButton_5_clicked()
{
    std::cout << "Initializing Viewer..." << std::endl;

    // 从yaml配置文件中读取
    string path_config = ui->lineEdit_configPath->text().toStdString();
    std::cout << "Load configuration from file : " << path_config << std::endl;
    cv::FileStorage config( path_config.c_str(), cv::FileStorage::READ );

    string path_setting = string(config["PATH.SystemSetting"]);
    string path_pcd = string(config["PATH.PCD"]);

    // 更新标签.
    msLabelConfigPath = string(config["PATH.LabelConfig"]);
    readLabelConfig(msLabelConfigPath);
    // 更新实例.
    msInstancesPath = string(config["PATH.Instance"]);;
    readInstances(msInstancesPath);
    refreshInstanceTable();

    // 读取地平面
    string strGroundPlane = string(config["Setting.Groundplane"]);
    mGroundPlane = loadPlaneFromString(strGroundPlane);

    std::cout << " - path_setting : " << path_setting << std::endl;
    std::cout << " - path_pcd : " << path_pcd << std::endl;
    std::cout << " - msInstancesPath : " << msInstancesPath << std::endl;
    std::cout << " - msLabelConfigPath : " << msLabelConfigPath << std::endl;
    std::cout << " - strGroundPlane : " << strGroundPlane << std::endl;

    mVisualizer.startVisualizationThread(path_setting);
    mVisualizer.addBackgroundPoint(path_pcd);
    mVisualizer.refreshInstances(mvInstances);
}


void MainWindow::on_pushButton_2_clicked()
{

}

void MainWindow::on_pushButton_6_clicked()
{
    // 入口: txt 文档 -> 内存

    // 添加: 改变内存, 重新渲染显示
    // 修改过程 -> 同步改变内存

    // 出口: 关闭程序, 内存存储到txt

    Instance ins;
    ins.id = mvInstances.size();
    ins.label = 0;

    mvInstances.push_back(ins);

    refreshInstanceTable(); // 刷新实例table

}

void MainWindow::on_tableWidget_instance3D_cellChanged(int row, int column)
{
    if( !mbInstanceTableInitiated ) return;
    if(mvInstances.size() == 0) return;
    // 更新内存中的Instance存储情况.
    if( column == INS_TABLE_LABEL ){
        QTableWidgetItem* item = ui->tableWidget_instance3D->item(row, column);

        // 先拆分  id / txt.
        QString str = item->text();
        QString num_str = str.section(" / ", 0,0);

        mvInstances[row].label = num_str.toInt();
    }
    else if( column == INS_TABLE_GPROT )
    {
        QTableWidgetItem* item = ui->tableWidget_instance3D->item(row, column);

        double rot_angle = item->text().toDouble();
        // 若是旋转改变了.

        // 注意需要保证二者的数量一致, id对应.
        AddRotToInstance(mvInstancesWithRot[row], mvInstances[row], rot_angle);

        // 此处不需要修改 mvInstances中的值.
    }
    else if( column > 0 ){
        QTableWidgetItem* item = ui->tableWidget_instance3D->item(row, column);
        mvInstances[row].param[column-1] = item->text().toDouble();

        QTableWidgetItem* item_rot_angle = ui->tableWidget_instance3D->item(row, INS_TABLE_GPROT);
        double rot_angle = item_rot_angle->text().toDouble();
        AddRotToInstance(mvInstancesWithRot[row], mvInstances[row], rot_angle);
    }

    // 计算 mvInstancesWithRot， 将地面法向量叠加的计算输出并显示.
    // refreshInstancesWithRot();

    // 更新
    if(mVisualizer.isInitialized())
        mVisualizer.refreshInstances(mvInstancesWithRot);

}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    double percentage = 1.0 + (double)(value-50)/100.0;
    double changed_value = miCurrentSliderCenter * percentage;

    QTableWidgetItem* item = ui->tableWidget_instance3D->item(miCurrentInstanceTableRow, miCurrentInstanceTableCol);
    item->setText(QString::number(changed_value));
}

void MainWindow::on_tableWidget_instance3D_currentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn)
{
    int row = currentRow;
    int column = currentColumn;
    if( column != INS_TABLE_GPROT && (column < INS_TABLE_X || column > INS_TABLE_C) ) return;
    // 获取当前值.
    QTableWidgetItem* item = ui->tableWidget_instance3D->item(row, column);
    double value = item->text().toDouble();

    miCurrentSliderCenter = value;

    miCurrentInstanceTableRow = row;
    miCurrentInstanceTableCol = column;

    std::cout << "miCurrentInstanceTableRow: " << miCurrentInstanceTableRow << std::endl;
    std::cout << "miCurrentInstanceTableCol: " << miCurrentInstanceTableCol << std::endl;
    // 刷新滚动条上下限
    ui->horizontalSlider->setValue(50);
}

void MainWindow::on_tableWidget_instance3D_cellActivated(int row, int column)
{

}

void MainWindow::on_pushButton_7_clicked()
{
    // Save all files.
    saveCurrentInstances();

    this->destroy();
}

void MainWindow::saveCurrentInstances()
{
    QTableWidget* pTable = ui->tableWidget_instance3D;

    int rows = pTable->rowCount();
    int cols = pTable->columnCount();

    int num = mvInstancesWithRot.size();

    // 保存
    MatrixXd insMat = instancesToMat(mvInstancesWithRot);

    // 保存mmDetMat
    saveMatToFile(insMat, (msInstancesPath+"_gt.txt").c_str());

}

MatrixXd MainWindow::instancesToMat(vector<Instance> &instances){
    int rows = instances.size();
    int cols = 11;
    Eigen::MatrixXd mat;
    mat.resize(rows, cols);
    for(int i=0;i<rows;i++)
    {
        Eigen::Matrix<double, 11, 1> vec;
        for(int n=0;n<9;n++)
            vec(n+1) = instances[i].param[n];
        vec(0) = instances[i].id;
        vec(10) = instances[i].label;

        mat.row(i) = vec;
    }

    return mat;

}

void MainWindow::automaticAssociation(){
    // 步骤1： 找到该帧对应的groundtruth;
    // 没有gt的帧可以直接舍去.

    // 临时参数： CALIB INFORMATION

    // 当前 timestamp

    // 感觉需要加入cv::Mat了. 然后将mLoader也交给 mAssociater去完成.
    VectorXd pose;
    string detFullPath = mDetectionFiles[miCurrentID];
    string bareFileName = splitFileName(detFullPath, true);
    string str_timestamp = bareFileName;
    bool find_gt = mLoader.getPoseFromTimeStamp(str_timestamp, pose); // 给TUM的loader添加一个函数接口.

    if(find_gt)
    {
        vector<Association> associations = mAssociater.process(pose, mmDetMat, true);

        // 获得了，如何显示出来?

        // 1） 直接将对应表格切换ID.
        int assoNum = associations.size();
        for(int i=0; i < assoNum; i++)
        {
            std::cout << "Asso " << i << " : " << associations[i].detID
                      << " -> " << associations[i].instanceID << std::endl;
            QString str;
            str = QString::number(associations[i].instanceID);
            ui->tableWidget->setItem(associations[i].detID, DET_TABLE_INSTANCE, new QTableWidgetItem(str));
        }

        // 2） 在图像中可视化投影结果，对应InstanceID， LabelID
        // cv 读取当前帧.
        // 读取图像切换.
        string imageFullPath = msImagePath + bareFileName + ".png";
        cv::Mat oriMat = cv::imread(imageFullPath, IMREAD_COLOR);

        cv::Mat mat = mAssociater.drawVisualization(oriMat, mmDetMat, associations);

        cv::Mat matRGB;
        cv::cvtColor(mat, matRGB, CV_BGR2RGB);
        (*pixmap) = QPixmap::fromImage(QImage((unsigned char*) matRGB.data, matRGB.cols, matRGB.rows, QImage::Format_RGB888));;

        palette->setBrush(frame->backgroundRole(),QBrush(*pixmap));
        frame->setPalette(*palette);
        frame->setAutoFillBackground(true);
        //frame->setMask(pixmap->mask());  //可以将图片中透明部分显示为透明的
        frame->show();

    }
    else
        std::cout << "No ground truth found for frame" << miCurrentID << std::endl;


}




void MainWindow::on_checkBox_showtraj_stateChanged(int arg1)
{
    if(ui->checkBox_showtraj->isChecked()){
        MatrixXd poseMat;
        poseMat.resize(0, 7);

        int jump = miTotalNum / 100;
        for( int i=0; i<miTotalNum ;i=i+jump)
        {
            // 开启
            VectorXd pose;
            cv::Mat rgb, depth;
            bool find_gt = mLoader.findFrameUsingID(i, rgb, depth, pose); // 给TUM的loader添加一个函数接口.

            if(find_gt)
            {
                poseMat.conservativeResize(poseMat.rows()+1, poseMat.cols());
                poseMat.row(poseMat.rows()-1) = pose;
            }

        }

        mVisualizer.addTrajectory(poseMat);
    }
    else
        mVisualizer.clearTrajectory();
        // 关闭
}

void MainWindow::AddRotToInstance(Instance &inWithRot, Instance &in, double rot_angle)
{
    // 先获得旋转矩阵
    g2o::ellipsoid e;
    Eigen::Matrix<double, 9, 1> v;
    for(int n=0;n<9;n++)
        v(n) = in.param[n];
    e.fromMinimalVector(v);

    Eigen::Quaterniond quat = e.pose.rotation();

    // 叠加旋转

    // 计算 rot axis.
    Eigen::AngleAxisd rotAxis(rot_angle, mGroundPlane.head(3));
    Eigen::Quaterniond rotQuat(rotAxis);

    Eigen::Quaterniond quatRoted = rotQuat * quat;   // 此处右乘 (0,0,1) 理论上也是一样的.

    e.setRotation(quatRoted);

    Eigen::Matrix<double, 9, 1> v_withRot;
    v_withRot = e.toMinimalVector();
    for(int n=0;n<9;n++)
        inWithRot.param[n] = v_withRot[n];


}

// void MainWindow::refreshInstancesWithRot()
// {
//     int num = mvInstances.size();
//     mvInstancesWithRot
// }

void MainWindow::on_pushButton_Jump_clicked()
{
    string sJumpID = ui->lineEdit_jump->text().toStdString();
    int goal_id = stoi(sJumpID);
    if(goal_id>0 && goal_id < miTotalNum){
        saveCurrentDetection();        // 保存当前情况

        miCurrentID = goal_id;
        dealWithPic(miCurrentID);
    }
}
