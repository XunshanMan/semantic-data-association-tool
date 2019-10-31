#ifndef DATASTRUCT_H
#define DATASTRUCT_H

/*
 * 定义一些数值标号.
 */

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
    INS_TABLE_X = 1,
    INS_TABLE_Y = 2,
    INS_TABLE_Z = 3,
    INS_TABLE_ROLL = 4,
    INS_TABLE_PITCH = 5,
    INS_TABLE_YAW = 6,
    INS_TABLE_A = 7,
    INS_TABLE_B = 8,
    INS_TABLE_C = 9,
    INS_TABLE_LABEL = 10,

};

#endif // DATASTRUCT_H
