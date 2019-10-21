
#include "utility.h"

#include <fstream>
#include <sstream>
#include <unistd.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <string.h> // for strcmp

#include <iostream>

#include <algorithm>
#include <fstream>

using namespace std;

bool compare_func_stringasdouble(string &s1, string &s2)
{
    string bareS1 = splitFileName(s1, true);
    string bareS2 = splitFileName(s2, true);
    return stod(bareS1) < stod(bareS2);
}

void GetFileNames(string path,vector<string>& filenames)
{
    filenames.clear();
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + "/" + ptr->d_name);
    }
    }
    closedir(pDir);

}

string splitFileName(string &s, bool bare)
{

    int pos = s.find_last_of('/');
    string name(s.substr(pos+1));

    if( bare )
    {
        // 取. 之前的文字
        int posBare = name.find_last_of('.');
        string bare_name(name.substr(0, posBare));
        return bare_name;
    }

    return name;

}

void sortFileNames(vector<string>& filenames, vector<string>& filenamesSorted)
{
    // 按double 类型排序.
    filenamesSorted = filenames;
    sort(filenamesSorted.begin(), filenamesSorted.end(), compare_func_stringasdouble);
}

vector<string> readStringFromFile(const char* fileName, bool dropFirstline){
    vector<string> strs;
    ifstream fin(fileName);
    string line;

    if(dropFirstline)
        getline(fin, line); // 扔掉第一行数据

    while( getline(fin, line) )
    {
        strs.push_back(line);
    }
    fin.close();

    return strs;

}
