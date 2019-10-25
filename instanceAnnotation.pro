#-------------------------------------------------
#
# Project created by QtCreator 2019-10-20T15:51:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = instanceAnnotation
TEMPLATE = app


SOURCES += main.cpp\
    instance.cpp \
        mainwindow.cpp \
    utility.cpp

HEADERS  += mainwindow.h \
    instance.h \
    utility.h \
    dataprocess_utils.h

FORMS    += mainwindow.ui

CONFIG += C++11
QMAKE_CXXFLAGS += -std=c++0x

# For opencv
INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui

# For Eigen
INCLUDEPATH += /usr/include/eigen3

# For data_process lib
LIBS += -L/home/jhz/workspace/projects/ellipsoid-slam/core/lib -lutils -lEllipsoidSLAM
INCLUDEPATH += /home/jhz/workspace/projects/ellipsoid-slam/core/include
INCLUDEPATH += /home/jhz/workspace/projects/ellipsoid-slam/core

DISTFILES += \
    CMakeLists.txt
