#include "VelocityControl.h"

VelocityControl::VelocityControl(float Kp, float Ki, float Kd, float tSample) {

    tSample_ = tSample;

    Kp_ = Kp; Ki_ = Ki; Kd_ = Kd;

    Iout = 0.0;
    prevIout = 0.0;

    Error = 0.0;
    prevError = 0.0;
    accError = 0.0;
}


float VelocityControl::calcPID(float Omega_ref, float Omega) {

    //偏差の計算
    Error = Omega_ref - Omega;

    //アンチワインドアップ 偏差の積分値の計算
    if (!(prevIout >= IMAX && Error > 0.0) && !(prevIout <= -IMAX && Error < 0.0)) {
        accError += (Error + prevError) / 2.0 * tSample_; //偏差の積分値の計算
    }

    //偏差の微分値の計算
    float diffError = (Error - prevError) / tSample_;

    //出力電圧値の計算
    Iout = Kp_ * Error + Ki_ * accError + Kd_ * diffError;

    //出力値、偏差の値の更新
    prevIout = Iout;
    prevError = Error;

    return Iout;
}




