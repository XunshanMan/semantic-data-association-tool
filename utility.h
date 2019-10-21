/*
 * 该文件负责处理与ubuntu 系统的文件操作
 */
#ifndef UTILITY_H
#define UTILITY_H

#include <vector>
#include <string>

using namespace std;

/*
 * 读取目录下的所有文件名.
 */
void GetFileNames(string path,vector<string>& filenames);
string splitFileName(string &s, bool bare = false);

/*
 * 将按double大小顺序排序。 从小到大。
 */
void sortFileNames(vector<string>& filenames, vector<string>& filenamesSorted);

vector<string> readStringFromFile(const char* fileName, bool dropFirstline = false);


#endif // UTILITY_H

