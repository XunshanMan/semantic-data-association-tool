#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utility.h"

#include <vector>
#include <string>

#include <iostream>

// 快捷键
#include <QShortcut>

#include "dataprocess_utils.h"
#include <Eigen/Core>

#include "instance.h"

using namespace Eigen;
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 初始化图像显示
    frame = new QFrame(this);
//    frame->resize(640,480);
    frame->setGeometry(850,10,640,480);
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
    miCurrentID = 408;  // DEBUG: 408物体比较多.
    dealWithPic(miCurrentID);

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
    MatrixXd instanceMat = readDataFromFile(path.c_str(), true);        // 剔除首行
    int num = instanceMat.rows();

    mvInstances.clear();
    mvInstances.resize(num);
    for( int i=0; i<num; i++)
    {
        VectorXd v = instanceMat.row(i);

        Instance obj;
        obj.id = int(v(0));
        obj.label = int(v(1));

        mvInstances[i] = obj;
    }


}

void MainWindow::refreshInstanceTable(){
    int num = mvInstances.size();
    QTableWidget* pTable = ui->tableWidget_instance;
    pTable->clearContents();
    pTable->setRowCount(num);
    for( int i=0; i<num;i++ )
    {
        Instance* pObj = &mvInstances[i];
        pTable->setItem(i, 0, new QTableWidgetItem(QString::number(pObj->id)));

        int label = pObj->label;
        QString str;
        str = QString::number(label) + QString(" / ") + QString::fromStdString(mvLabelIDToString[label]);
        pTable->setItem(i, 1, new QTableWidgetItem(str));

    }
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
