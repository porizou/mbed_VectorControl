#ifndef __CurrentControl_h
#define __CurrentControl_h

#include "mbed.h"

#define VMAX 12.0 //出力電圧値の最大値[V]

class CurrentControl {
public:
    /**
    * Kp 比例ゲイン
    * Ki 積分ゲイン
    * Kd 微分ゲイン
    * tSample 制御周期[s]
    */
    CurrentControl(float Kp, float Ki, float Kd, float tSample);

    /**
    * Iref 電流値の目標値[A]
    * I    電流値の現在値[A]
    */
    float calcPID(float Iref, float I); //PIDの計算

private:
    //PIDゲイン
    float Kp_, Ki_, Kd_;

    float tSample_; //制御周期[s]

    float     Vout; //出力電圧[V]
    float prevVout; //前回の出力電圧[V]

    float     Error; //現在の偏差
    float prevError; //前回の偏差

    float accError; //偏差の積分値
};


#endif