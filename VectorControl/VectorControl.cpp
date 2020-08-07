#include "VectorControl.h"


VectorControl::VectorControl(PinName upper_U, PinName upper_V, PinName upper_W,
              PinName lower_U, PinName lower_V, PinName lower_W,
              PinName Uplus, PinName Uminus,
              PinName Vplus, PinName Vminus): 
    pwm(upper_U, upper_V, upper_W, lower_U, lower_V, lower_W,
        FREQ, DEADTIME,
        Uplus, Uminus, Vplus, Vminus) 
{
        Iu = 0.0;
        Iv = 0.0;
        Iw = 0.0;

        Id = 0.0;
        Iq = 0.0;

        Vu = 0.0;
        Vv = 0.0;
        Vw = 0.0;

        // αβ座標変換行列の初期化
        float r2, r3;

        r2 = sqrt(2.0); r3 = sqrt(3.0);

        Cuvw[0][0] = r2/r3; Cuvw[0][1] = -1.0/r2/r3; Cuvw[0][2] = -1.0/r2/r3;
        Cuvw[1][0] = 0;     Cuvw[1][1] = 1.0/r2;     Cuvw[1][2] = -1.0/r2;
}

void VectorControl::calcIdq(float Theta) {
    //αβ座標からdq座標への変換行列の設定
    float c, s;

    c = cos(Theta); s = sin(Theta);

    float Cdq[2][2];

    Cdq[0][0] = c;  Cdq[0][1] = s; //Cdq = |  cosθ  sinθ  |
    Cdq[1][0] = -s; Cdq[1][1] = c; //      | -sinθ  cosθ  |

    //3相電流値の取得
    Iu = pwm.getIu(); Iv = pwm.getIv(); Iw = -Iu - Iv; //Iu + Iv + Iw = 0

    //3相座標電流値をαβ座標電流値に変換
    float Ialfa, Ibeta;

    Ialfa = (Cuvw[0][0] - Cuvw[0][1]) * Iu;
    Ibeta = Cuvw[1][1] * (Iu + 2 * Iv);

    //αβ座標電流値をdq座標電流値に変換

    Id = Cdq[0][0] * Ialfa + Cdq[0][1] * Ibeta;
    Iq = Cdq[1][0] * Ialfa + Cdq[1][1] * Ibeta;
}

float VectorControl::getId(void) {
    return Id;
}

float VectorControl::getIq(void) {
    return Iq;
}

float VectorControl::getIu(void) {
    return Iu;
}

float VectorControl::getIv(void) {
    return Iv;
}

float VectorControl::getIw(void) {
    return Iw;
}



void VectorControl::calcVuvw(float Vd, float Vq, float Theta) {

    //αβ座標からdq座標への変換行列の設定
    float c, s;

    c = cos(Theta); s = sin(Theta);

    float Cdq[2][2];

    Cdq[0][0] = c;  Cdq[0][1] = s; //Cdq = |  cosθ  sinθ  |
    Cdq[1][0] = -s; Cdq[1][1] = c; //      | -sinθ  cosθ  |

    //dq座標電圧からαβ座標電圧を計算
    float Valfa, Vbeta;

    Valfa = Cdq[0][0] * Vd + Cdq[1][0] * Vq;
    Vbeta = Cdq[0][1] * Vd + Cdq[1][1] * Vq;

    //αβ座標電圧からモータに印加するUVW座標電圧を計算
    Vu = Cuvw[0][0] * Valfa;
    Vv = Cuvw[0][1] * Valfa + Cuvw[1][1] * Vbeta;
    Vw = -Vu -Vv;
}

float VectorControl::getVu(void) {
    return Vu;
}

float VectorControl::getVv(void) {
    return Vv;
}

float VectorControl::getVw(void) {
    return Vw;
}


void VectorControl::outPWM(void) {
    float duty_u, duty_v, duty_w;

    //duty比を計算
    duty_u = Vu / VdqMAX + 0.5;
    duty_v = Vv / VdqMAX + 0.5;
    duty_w = Vw / VdqMAX + 0.5;

    //duty比をPWMクラスに渡す
    pwm.setUVW(duty_u, duty_v, duty_w);
}

void VectorControl::start(void) {
    pwm.startPWM();
}

void VectorControl::stop(void) {
    pwm.stopPWM();
}















