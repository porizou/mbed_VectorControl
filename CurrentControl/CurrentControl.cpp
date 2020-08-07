#include "CurrentControl.h"

CurrentControl::CurrentControl(float Kp, float Ki, float Kd, float tSample) {

    tSample_ = tSample;

    Kp_ = Kp; Ki_ = Ki; Kd_ = Kd;

    Vout = 0.0;
    prevVout = 0.0;

    Error = 0.0;
    prevError = 0.0;
    accError = 0.0;
}


float CurrentControl::calcPID(float Iref, float I) {

    //偏差の計算
    Error = Iref - I; 

    //アンチワインドアップ 偏差の積分値の計算
    if (!(prevVout >= VMAX && Error > 0.0) && !(prevVout <= -VMAX && Error < 0.0)) {
        accError += (Error + prevError) / 2.0 * tSample_; //偏差の積分値の計算
    }

    //偏差の微分値の計算
    float diffError = (Error - prevError) / tSample_;

    //出力電圧値の計算
    Vout = Kp_ * Error + Ki_ * accError + Kd_ * diffError;

    //出力値、偏差の値の更新
    prevVout = Vout;
    prevError = Error;

    return Vout;
}




