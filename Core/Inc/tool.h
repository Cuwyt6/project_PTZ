//
// Created by lenovo on 24-12-1.
//

#ifndef PTZ_TOOL_H
#define PTZ_TOOL_H
float limit(float x, float max);

template<typename T1, typename T2>
T2 LinearMap(T1 value, T1 in_min, T1 in_max, T2 out_min, T2 out_max) {
    if (in_min == in_max) {
        return out_min; // 防止除以零
    }
    return out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min);
}

#endif //PTZ_TOOL_H
