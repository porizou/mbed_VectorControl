#include "3PhasePWM.h"

#include "mbed.h"

ThreePhasePWM::ThreePhasePWM(PinName upper_U, PinName upper_V, PinName upper_W,
          PinName lower_U, PinName lower_V, PinName lower_W,
          float Frequency, float DeadTime, 
          PinName Uplus, PinName Uminus,
          PinName Vplus, PinName Vminus):
          pwm_upper_U(upper_U), pwm_upper_V(upper_V), pwm_upper_W(upper_W),
          pwm_lower_U(lower_U), pwm_lower_V(lower_V), pwm_lower_W(lower_W),
          VshuntR_Uplus(Uplus), VshuntR_Uminus(Uminus),
          VshuntR_Vplus(Vplus), VshuntR_Vminus(Vminus)
{
    pwm_upper_U = 0;
    pwm_upper_V = 0;
    pwm_upper_W = 0;
    pwm_lower_U = 0;
    pwm_lower_V = 0;
    pwm_lower_W = 0;

    Frequency_ = Frequency;
    DeadTime_  = DeadTime;
    
    Iu = 0.0;
    Iv = 0.0;
}


#ifdef TOOTHWAVE
void ThreePhasePWM::PwmUout(void) {
    unsigned char i = 0;//i=0のときU相
    uvw[i].mode += 1;   //チョッピングのオンオフを決定するモードを１増やす

    if(uvw[i].mode == 1) { // モードが1のとき、Tonの状態をつくる
        pwm_upper_U = 1; //上アームUuをON
        pwm_lower_U = 0; //下アームUdをOFF

        // モード1の時間幅 T1 = Ton-Tdt を計算
        uvw[i].upper_us = uvw[i].duty * 1000000 / Frequency_ - DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].upper_us < TMIN ) uvw[i].upper_us = TMIN;

        // T1[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, uvw[i].upper_us);

        // モード3の時間幅 T3=Toff-Tdt=Tpwm-(T1+Tdt)-Tdtを計算
        uvw[i].lower_us = 1000000 / Frequency_ - uvw[i].upper_us - 2 * DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].lower_us < TMIN ) uvw[i].lower_us = TMIN;
    }
    else if(uvw[i].mode == 2) { // モードが２のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, DeadTime_);

        pwm_upper_U = 0;         // 上アームUuをオフ
        pwm_lower_U = 0;         // 下アームUdをオフ
    }
    else if(uvw[i].mode == 3) { // モードが３のとき、Toffの状態をつくる
        // T3[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, uvw[i].lower_us);

        pwm_upper_U = 0;         // 上アームUuをオフ
        pwm_lower_U = 1;         // 下アームUdをオン
    }
    else {                     // モードが４のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, DeadTime_);
        
        // シャント抵抗の両端の電圧を見てモータ電流を検出
        Iu = -(VshuntR_Uplus - VshuntR_Uminus)*5.0 /R_SHUNT; // iu [A]

        pwm_upper_U = 0;         // 上アームUuをオフ
        pwm_lower_U = 0;         // 下アームUdをオフ

        uvw[i].mode = 0;          // チョッピングのオンオフを決定するモードを0にする
    }
}

void ThreePhasePWM::PwmVout(void) {
    unsigned char i = 1;//i=1のときV相
    uvw[i].mode += 1;   //チョッピングのオンオフを決定するモードを１増やす

    if(uvw[i].mode == 1) { // モードが1のとき、Tonの状態をつくる
        pwm_upper_V = 1; //上アームVuをON
        pwm_lower_V = 0; //下アームVdをOFF

        // モード1の時間幅 T1 = Ton-Tdt を計算
        uvw[i].upper_us = uvw[i].duty * 1000000 / Frequency_ - DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].upper_us < TMIN ) uvw[i].upper_us = TMIN;

        // T1[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, uvw[i].upper_us);

        // モード3の時間幅 T3=Toff-Tdt=Tpwm-(T1+Tdt)-Tdtを計算
        uvw[i].lower_us = 1000000 / Frequency_ - uvw[i].upper_us - 2 * DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].lower_us < TMIN ) uvw[i].lower_us = TMIN;
    }
    else if(uvw[i].mode == 2) { // モードが２のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, DeadTime_);

        pwm_upper_V = 0;         // 上アームVuをオフ
        pwm_lower_V = 0;         // 下アームVdをオフ
    }
    else if(uvw[i].mode == 3) { // モードが３のとき、Toffの状態をつくる
        // T3[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, uvw[i].lower_us);

        pwm_upper_V = 0;         // 上アームVuをオフ
        pwm_lower_V = 1;         // 下アームVdをオン
    }
    else {                     // モードが４のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, DeadTime_);
        
        // シャント抵抗の両端の電圧を見てモータ電流を検出
        Iv = -(VshuntR_Vplus - VshuntR_Vminus)*5.0 /R_SHUNT; // iv [A]

        pwm_upper_V = 0;         // 上アームVuをオフ
        pwm_lower_V = 0;         // 下アームVdをオフ

        uvw[i].mode = 0;          // チョッピングのオンオフを決定するモードを0にする
    }
}

void ThreePhasePWM::PwmWout(void) {
    unsigned char i = 2;//i=2のときW相
    uvw[i].mode += 1;   //チョッピングのオンオフを決定するモードを１増やす

    if(uvw[i].mode == 1) { // モードが1のとき、Tonの状態をつくる
        pwm_upper_W = 1; //上アームWuをON
        pwm_lower_W = 0; //下アームWdをOFF

        // モード1の時間幅 T1 = Ton-Tdt を計算
        uvw[i].upper_us = uvw[i].duty * 1000000 / Frequency_ - DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].upper_us < TMIN ) uvw[i].upper_us = TMIN;

        // T1[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, uvw[i].upper_us);

        // モード3の時間幅 T3=Toff-Tdt=Tpwm-(T1+Tdt)-Tdtを計算
        uvw[i].lower_us = 1000000 / Frequency_ - uvw[i].upper_us - 2 * DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].lower_us < TMIN ) uvw[i].lower_us = TMIN;
    }
    else if(uvw[i].mode == 2) { // モードが２のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, DeadTime_);

        pwm_upper_W = 0;         // 上アームWuをオフ
        pwm_lower_W = 0;         // 下アームWdをオフ
    }
    else if(uvw[i].mode == 3) { // モードが３のとき、Toffの状態をつくる
        // T3[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, uvw[i].lower_us);

        pwm_upper_W = 0;         // 上アームWuをオフ
        pwm_lower_W = 1;         // 下アームWdをオン
    }
    else {                     // モードが４のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, DeadTime_);

        pwm_upper_W = 0;         // 上アームWuをオフ
        pwm_lower_W = 0;         // 下アームWdをオフ

        uvw[i].mode = 0;          // チョッピングのオンオフを決定するモードを0にする
    }
}
#endif


#ifdef TRIANGLERWAVE
void ThreePhasePWM::PwmUout(void) {
    unsigned char   i = 0;  // i=0のときU相
    uvw[i].mode += 1;   //チョッピングのオンオフを決定するモードを１増やす

    if( uvw[i].mode == 1 ){ // モードが1のとき、Toffの状態をつくる
        pwm_upper_U = 0;  // 上アームUuをオフ
        pwm_lower_U = 1;  // 下アームUdをオン

        // モード3の時間幅 T3 = Ton-Tdt を計算
        uvw[i].upper_us = uvw[i].duty * 1000000 / Frequency_ - DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].upper_us < TMIN ) uvw[i].upper_us = TMIN;

        // モード1,5の時間幅 T1=(Toff-Tdt)/2=(Tpwm-T3-2Tdt)/2を計算
        uvw[i].lower_us = (1000000 / Frequency_ - uvw[i].upper_us - 2 * DeadTime_) / 2.0;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].lower_us < TMIN ) uvw[i].lower_us = TMIN;

        // T1[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, uvw[i].lower_us);

    }
    else if( uvw[i].mode == 2 ) { // モードが２のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, DeadTime_);

        pwm_upper_U = 0;         // 上アームUuをオフ
        pwm_lower_U = 0;         // 下アームUdをオフ
    }
    else if( uvw[i].mode == 3 ) { // モードが３のとき、Tonの状態をつくる
        // T3[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, uvw[i].upper_us);

        pwm_upper_U = 1;         // 上アームUuをオン
        pwm_lower_U = 0;         // 下アームUdをオフ
    }
    else if( uvw[i].mode == 4 ) { // モードが４のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, DeadTime_);

        pwm_upper_U = 0;         // 上アームUuをオフ
        pwm_lower_U = 0;         // 下アームUdをオフ
    }
    else {                     // モードが５のとき、Toffの状態をつくる
        // T5(=T1)[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmUout, uvw[i].lower_us);

        pwm_upper_U = 0;         // 上アームUuをオフ
        pwm_lower_U = 1;         // 下アームUdをオン

        uvw[i].mode = 0;          // チョッピングのオンオフを決定するモードを0にする
    }                            
}

void ThreePhasePWM::PwmVout(void) {
    unsigned char   i = 1;  // i=1のときV相
    uvw[i].mode += 1;   //チョッピングのオンオフを決定するモードを１増やす

    if( uvw[i].mode == 1 ){ // モードが1のとき、Toffの状態をつくる
        pwm_upper_V = 0;  // 上アームVuをオフ
        pwm_lower_V = 1;  // 下アームVdをオン

        // モード3の時間幅 T3 = Ton-Tdt を計算
        uvw[i].upper_us = uvw[i].duty * 1000000 / Frequency_ - DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].upper_us < TMIN ) uvw[i].upper_us = TMIN;

        // モード1,5の時間幅 T1=(Toff-Tdt)/2=(Tpwm-T3-2Tdt)/2を計算
        uvw[i].lower_us = (1000000 / Frequency_ - uvw[i].upper_us - 2 * DeadTime_) / 2.0;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].lower_us < TMIN ) uvw[i].lower_us = TMIN;

        // T1[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, uvw[i].lower_us);

    }
    else if( uvw[i].mode == 2 ) { // モードが２のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, DeadTime_);

        pwm_upper_V = 0;         // 上アームVuをオフ
        pwm_lower_V = 0;         // 下アームVdをオフ
    }
    else if( uvw[i].mode == 3 ) { // モードが３のとき、Tonの状態をつくる
        // T3[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, uvw[i].upper_us);

        pwm_upper_V = 1;         // 上アームVuをオン
        pwm_lower_V = 0;         // 下アームVdをオフ
    }
    else if( uvw[i].mode == 4 ) { // モードが４のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, DeadTime_);

        pwm_upper_V = 0;         // 上アームVuをオフ
        pwm_lower_V = 0;         // 下アームVdをオフ
    }
    else {                     // モードが５のとき、Toffの状態をつくる
        // T5(=T1)[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmVout, uvw[i].lower_us);

        pwm_upper_V = 0;         // 上アームVuをオフ
        pwm_lower_V = 1;         // 下アームVdをオン

        uvw[i].mode = 0;          // チョッピングのオンオフを決定するモードを0にする
    }         
}

void ThreePhasePWM::PwmWout(void) {
    unsigned char   i = 2;  // i=2のときW相
    uvw[i].mode += 1;   //チョッピングのオンオフを決定するモードを１増やす

    if( uvw[i].mode == 1 ){ // モードが1のとき、Toffの状態をつくる
        pwm_upper_W = 0;  // 上アームWuをオフ
        pwm_lower_W = 1;  // 下アームWdをオン

        // モード3の時間幅 T3 = Ton-Tdt を計算
        uvw[i].upper_us = uvw[i].duty * 1000000 / Frequency_ - DeadTime_;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].upper_us < TMIN ) uvw[i].upper_us = TMIN;

        // モード1,5の時間幅 T1=(Toff-Tdt)/2=(Tpwm-T3-2Tdt)/2を計算
        uvw[i].lower_us = (1000000 / Frequency_ - uvw[i].upper_us - 2 * DeadTime_) / 2.0;
        // 時間幅が小さいときはTMINにする
        if( uvw[i].lower_us < TMIN ) uvw[i].lower_us = TMIN;

        // T1[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, uvw[i].lower_us);

    }
    else if( uvw[i].mode == 2 ) { // モードが２のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, DeadTime_);

        pwm_upper_W = 0;         // 上アームWuをオフ
        pwm_lower_W = 0;         // 下アームWdをオフ
    }
    else if( uvw[i].mode == 3 ) { // モードが３のとき、Tonの状態をつくる
        // T3[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, uvw[i].upper_us);

        pwm_upper_W = 1;         // 上アームWuをオン
        pwm_lower_W = 0;         // 下アームWdをオフ
    }
    else if( uvw[i].mode == 4 ) { // モードが４のとき、デッドタイムをつくる
        // Tdt[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, DeadTime_);

        pwm_upper_W = 0;         // 上アームWuをオフ
        pwm_lower_W = 0;         // 下アームWdをオフ
    }
    else {                     // モードが５のとき、Toffの状態をつくる
        // T5(=T1)[μs]経過してからタイムアウトでこの関数自身をコール
        pwm[i].attach_us(this, &ThreePhasePWM::PwmWout, uvw[i].lower_us);

        pwm_upper_W = 0;         // 上アームWuをオフ
        pwm_lower_W = 1;         // 下アームWdをオン

        uvw[i].mode = 0;          // チョッピングのオンオフを決定するモードを0にする
    }         
}
#endif



void ThreePhasePWM::startPWM(void) {

    for (int i = 0; i < 3; i++) {
        uvw[i].duty  = 0.5; // 0.5のときにVu=0[V]
        uvw[i].mode = 0;    // チョッピングのオンオフを決定するモードを初期化
    }

    //PWM 出力開始
    PwmUout();
    PwmVout();
    PwmWout();
}

void ThreePhasePWM::stopPWM(void) {
    for(int i = 0; i < 3; i++) {
        uvw[i].mode = 0;
        pwm[i].detach(); //タイマー割り込みを停止
    }
    pwm_upper_U = 0;
    pwm_upper_V = 0;
    pwm_upper_W = 0;
    pwm_lower_U = 0;
    pwm_lower_V = 0;
    pwm_lower_W = 0;
}



void ThreePhasePWM::setU(float duty_u) {
    uvw[0].duty = duty_u;
}

void ThreePhasePWM::setV(float duty_v) {
    uvw[1].duty = duty_v;
}

void ThreePhasePWM::setW(float duty_w) {
    uvw[2].duty = duty_w;
}

void ThreePhasePWM::setUVW(float duty_u, float duty_v, float duty_w) {
    uvw[0].duty = duty_u;
    uvw[1].duty = duty_v;
    uvw[2].duty = duty_w;
}

float ThreePhasePWM::getIu(void) {
    return Iu;
}

float ThreePhasePWM::getIv(void) {
    return Iv;
}

