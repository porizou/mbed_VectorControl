#ifndef __VelocityControl_h
#define __VelocityControl_h

#include "mbed.h"

#define IMAX 10 //出力電流値の最大値[A]

class VelocityControl {
public:
    /**
    * Kp 比例ゲイン
    * Ki 積分ゲイン
    * Kd 微分ゲイン
    * tSample 制御周期[s]
    */
    VelocityControl(float Kp, float Ki, float Kd, float tSample);

    /**
    * Omega_ref 速度の目標値[rad/s]
    * Omega     速度の現在値[rad/s]
    */
    float calcPID(float Omega_ref, float Omega); //PIDの計算

private:
    //PIDゲイン
    float Kp_, Ki_, Kd_;

    float tSample_; //制御周期[s]

    float     Iout; //出力電流値[A]
    float prevIout; //前回の出力電流値[A]

    float     Error; //現在の偏差
    float prevError; //前回の偏差

    float accError; //偏差の積分値
};


#endif