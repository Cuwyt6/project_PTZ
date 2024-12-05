//
// Created by lenovo on 24-11-21.
//该文件用于存放数学工具，例如limit（上下限函数），线性映射函数等等

#include <cmath>
#include "tool.h"

float limit(float x, float max){
    return (fabsf(x) < fabsf(max)) ? x : max;
}

