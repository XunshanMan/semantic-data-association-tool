#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"

#include <fstream>
#include <sstream>
#include <unistd.h>
#include <dirent.h>

#include <string>
#include <vector>

#include <Eigen/Core>

#include "instance.h"
#include "visualize.h"
#include "DataStruct.h"
#include "autodataassociation.h"

// TUM RGBD数据集处理
#include "src/tum_rgbd/io.h"

using namespace std;
using namespace Eigen;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_left_clicked();

    void on_pushButton_right_clicked();

    void on_tableWidget_itemSelectionChanged();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_tableWidget_instance_itemSelectionChanged();

    void on_pushButton_5_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_6_clicked();

    void on_tableWidget_instance3D_cellChanged(int row, int column);

    void on_tableWidget_instance3D_cellActivated(int row, int column);

    void on_horizontalSlider_valueChanged(int value);

    void on_tableWidget_instance3D_currentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn);

    void on_pushButton_7_clicked();

    void on_checkBox_showtraj_stateChanged(int arg1);

private:
    Ui::MainWindow *ui;
    QFrame *frame;
    QPixmap *pixmap;
    QPalette *palette;

    // 记录文件夹内的文件名
    std::vector<string> mDetectionFiles;

    int miCurrentID;    // 当前处理帧数
    int miTotalNum;     // 总共待处理帧数

    string msDetectPath;
    string msImagePath;
    string msLabelConfigPath;
    string msInstancesPath;
    string msDatasetDir;

    vector<string> mvLabelIDToString;

    vector<Instance> mvInstances;

    MatrixXd mmDetMat;

    visualizer mVisualizer;

    AutoDataAssociation mAssociater;

    bool mbInstanceTableInitiated;

    // 实现滚动条功能
    double miCurrentSliderCenter;
    int miCurrentInstanceTableRow;
    int miCurrentInstanceTableCol;

    // 数据集处理器
    TUMRGBD::Dataset mLoader;

private:
// 一些函数
    void refreshText();

    void dealWithPic(int id);
    void readDetection(int id);
    void refreshTable(MatrixXd &mat);   // 刷新检测表格


    void readLabelConfig(string &path);
    void readInstances(string &path);

    void refreshInstanceTable();   // 刷新实例表格

    // 保存文件
    void saveCurrentDetection();
    void saveCurrentInstances();

    // 可视化处理
    void Visualization(string& path_setting);

    MatrixXd instancesToMat(vector<Instance> &instances);

    void automaticAssociation();

};

#endif // MAINWINDOW_H
