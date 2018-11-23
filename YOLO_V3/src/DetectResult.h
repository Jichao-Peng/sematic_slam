//
// Created by leo on 18-11-20.
//

#ifndef PROJECT_DETECTRESULT_H
#define PROJECT_DETECTRESULT_H

#include <string>

using namespace std;

//把这个类单独放一个h文件是因为Frame类的编译链接问题
class DetectResult
{
public:
    string mName;
    float mConfidence;
    float mTop;
    float mBottom;
    float mLeft;
    float mRight;
};

#endif //PROJECT_DETECTRESULT_H
