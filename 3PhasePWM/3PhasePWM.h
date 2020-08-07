#ifndef __3PhasePWM_h
#define __3PhasePWM_h

#include "mbed.h"


/* 
 *キャリアの波形の指定
 *TOOTHWAVEかTRIANGLERWAVEのどちらかを定義
*/
#define TOOTHWAVE  //キャリアの波形 ノコギリ波
//#define TRIANGLERWAVE //キャリアの波形 三角波

#define TMIN 3 //PWM時間幅の最小値 [μs]

#define R_SHUNT     1000   // 電流検出用シャント抵抗の値[Ω]




class ThreePhasePWM {


public:
    ThreePhasePWM(PinName upper_U, PinName upper_V, PinName upper_W,
              PinName lower_U, PinName lower_V, PinName lower_W,
              float Frequency, float DeadTime,
              PinName Uplus, PinName Uminus,
              PinName Vplus, PinName Vminus);
    /*
     * upper_U U相上アームディジタル出力ポート
     * upper_V V相上アームディジタル出力ポート
     * upper_W W相上アームディジタル出力ポート
     * lower_U U相下アームディジタル出力ポート
     * lower_V V相下アームディジタル出力ポート
     * lower_W W相下アームディジタル出力ポート
     * Frequency PWMキャリア周波数[Hz]
     * deadtime  デッドタイムの時間幅[μs]
     *
     * Uplus 電流センサU相+側アナログ入力ポート
     * Uminus 電流センサU相-側アナログ入力ポート
     * Vplus 電流センサV相+側アナログ入力ポート
     * Vminus 電流センサV相-側アナログ入力ポート
     */

    void startPWM(void); //PWM開始
    void stopPWM(void);  //PWM停止


    void setU(float duty_u); //U相Duty比を設定
    void setV(float duty_v); //V相Duty比を設定
    void setW(float duty_w); //W相Duty比を設定
    void setUVW(float duty_u, float duty_v , float duty_w); //3相Duty比を設定
    
    float getIu(void);
    float getIv(void);


private:

    void PwmUout(void); //U相PWM出力関数
    void PwmVout(void); //V相PWM出力関数
    void PwmWout(void); //W相PWM出力関数

    DigitalOut pwm_upper_U; //U相上アームディジタル出力ポート
    DigitalOut pwm_upper_V; //V相上アームディジタル出力ポート
    DigitalOut pwm_upper_W; //W相上アームディジタル出力ポート
    DigitalOut pwm_lower_U; //U相下アームディジタル出力ポート
    DigitalOut pwm_lower_V; //V相下アームディジタル出力ポート
    DigitalOut pwm_lower_W; //W相下アームディジタル出力ポート
    
    AnalogIn VshuntR_Uplus;  // 3.3[V], 電流センサU相+側アナログ入力
    AnalogIn VshuntR_Uminus; // 3.3[V], 電流センサU相-側アナログ入力
    
    AnalogIn VshuntR_Vplus;  // 3.3[V], 電流センサV相+側アナログ入力
    AnalogIn VshuntR_Vminus;  // 3.3[V], 電流センサV相-側アナログ入力

    Timeout pwm[3]; //タイムアウト関数の宣言

    float Frequency_; //PWMキャリア周波数 F[Hz]
    float DeadTime_;     //デットタイム Tdt [μs]

    typedef struct struct_PWM_param{
        float duty;         // 0.0-1.0, PWMデューティ比
        unsigned char mode; // チョッピングのON,OFFを決定するモード
        long  upper_us;     // 上側アームをONにする時間幅[μs]
        long  lower_us;     // 下側アームをONにする時間幅[μs]
    }PWMparam;
    
    PWMparam uvw[3];
    
    float Iu;
    float Iv;
};

#endif 
//3PhasePWM_H