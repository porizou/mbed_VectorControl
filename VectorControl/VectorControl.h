#ifndef __VectorControl_h
#define __VectorControl_h

#include "mbed.h"
#include "3PhasePWM.h"

#define VdqMAX 12 //Vdqの大きさの最大値
#define SQR_VdqMAX (VdqMAX * VdqMAX) //Vdqの大きさの最大値の2乗

#define DEADTIME 0.00005 //デッドタイム[s]
#define FREQ 1000.0      //PWMキャリア周波数 [Hz]


class VectorControl {

public:
    VectorControl(PinName upper_U, PinName upper_V, PinName upper_W,
              PinName lower_U, PinName lower_V, PinName lower_W,
              PinName Uplus, PinName Uminus,
              PinName Vplus, PinName Vminus);

    void calcIdq(float Theta); //3相電流からdq軸電流の座標変換

    float getId(void); //d軸電流の取得
    float getIq(void); //q軸電流の取得

    float getIu(void); //U相電流の取得
    float getIv(void); //V相電流の取得
    float getIw(void); //W相電流の取得

    void calcVuvw(float Vd, float Vq, float Theta); //dq軸電圧から3相電圧の座標変換

    float getVu(void); //U相電圧の取得
    float getVv(void); //V相電圧の取得
    float getVw(void); //W相電圧の取得

    void outPWM(void); //3相PWMの出力

    void start(void); //PWM出力開始
    void stop(void);  //PWM出力停止


private:
    ThreePhasePWM pwm; //3相PWMクラス

    float Iu, Iv, Iw; //3相電流 [A]
    float Id, Iq; //dq軸電流 [A]

    float Vu, Vv, Vw; //3相電圧[V]

    float Cuvw[2][3]; //dq-αβ座標変換行列

};




#endif 
