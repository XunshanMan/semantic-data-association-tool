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

using namespace std;
using namespace Eigen;

namespace Ui {
class MainWindow;
}

// 表格ENUM
enum DETCTION_TABLE_COLS
{
    DET_TABLE_ID = 0,
    DET_TABLE_X1 = 1,
    DET_TABLE_Y1 = 2,
    DET_TABLE_X2 = 3,
    DET_TABLE_Y2 = 4,
    DET_TABLE_LABEL = 5,
    DET_TABLE_PROB = 6,
    DET_TABLE_INSTANCE = 7,
};

enum INSTANCE_TABLE_COLS
{
    INS_TABLE_ID = 0,
    INS_TABLE_LABEL = 1,

};

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

    vector<string> mvLabelIDToString;

    vector<Instance> mvInstances;

    MatrixXd mmDetMat;

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

};

#endif // MAINWINDOW_H
