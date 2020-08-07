#include "mbed.h"
#include "rtos.h"


#include "Encoder.h"
#include "VectorControl.h"
#include "VelocityControl.h"
#include "CurrentControl.h"

/* エンコーダの設定 */
#define ENC_A p11
#define ENC_B p12

#define ENC_NUM 20 //エンコーダの一回転あたりのパルス数

#define VELOCITYSAMPLE 0.001 //回転速度のサンプル周期[s]
#define THETASAMPLE 0.001 //回転角度のサンプル周期[s]

/* ベクトル制御の設定 */
#define U_U LED2
#define U_V p7
#define U_W p5
#define L_U LED4
#define L_V p9
#define L_W p10

#define U_PLUS  p16 // U相電流検出用抵抗の+側アナログ入力
#define U_MINUS p17 // U相電流検出用抵抗の-側アナログ入力
#define V_PLUS  p19 // V相電流検出用抵抗の+側アナログ入力
#define V_MINUS p20 // V相電流検出用抵抗の-側アナログ入力

/* 電流制御の設定 */
#define iKp 1.0 //電流制御Pゲイン
#define iKi 1.0 //電流制御Iゲイン
#define iKd 0.0 //電流制御Dゲイン

#define I_CONTROL_CYCLE 0.001 //電流制御周期


/* 速度制御の設定 */
#define vKp 1.0 //速度制御Pゲイン
#define vKi 1.0 //速度制御Iゲイン
#define vKd 0.0 //速度制御Dゲイン

#define V_CONTROL_CYCLE 0.002 //速度制御周期

Serial pc(USBTX, USBRX); // tx, rx

Encoder Enc(ENC_A, ENC_B, ENC_NUM, THETASAMPLE, VELOCITYSAMPLE);

VectorControl Vector(U_U, U_V, U_W, L_U, L_V, L_W, U_PLUS, U_MINUS, V_PLUS, V_MINUS);

CurrentControl  IdC(iKp, iKi, iKd, I_CONTROL_CYCLE); //d軸電流制御クラス
CurrentControl  IqC(iKp, iKi, iKd, I_CONTROL_CYCLE); //q軸電流制御クラス

VelocityControl VC(vKp, vKi, vKd, V_CONTROL_CYCLE); //速度制御クラス

DigitalOut led(LED1);



float Iqref = 0.0; //q軸電流目標値[A]
float Idref = 0.0; //d軸電流目標値[A]

float Vref = 0.0; //速度制御目標値[rad/s]


/* 電流制御スレッド */
void CurrentThread(void const *argument) {

    float Vd, Vq;
    float Theta;

    Theta = Enc.getAngle(); //回転角度の取得

    Vector.calcIdq(Theta); //dq軸電流値の取得

    Vd = IdC.calcPID(Idref, Vector.getId()); //d軸電流制御
    Vq = IqC.calcPID(Iqref, Vector.getIq()); //q軸電流制御

    Vector.calcVuvw(Vd, Vq, Theta); //3相電圧値の計算

    Vector.outPWM(); //PWMの出力
}

/* 速度制御スレッド */
void VelocityThread(void const *argument) {

    float V = Enc.getVelocity();

    Iqref = VC.calcPID(Vref, V);
}

int main() {
    //RTOSタイマーの宣言
    RtosTimer Current(CurrentThread);   //電流制御
    RtosTimer Velocity(VelocityThread); //速度制御

    Current.start(I_CONTROL_CYCLE * 1000);
    Velocity.start(V_CONTROL_CYCLE * 1000);
        
    while(1) {
    
        led = !led;
    }
}




























