#include "foc.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
float voltage_power_supply;
// float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
#define PI 3.1415926
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f

float S0_zero_electric_angle = 0;
int M0_PP = 1, M0_DIR = 1;
int M0_pwmA = 32;
int M0_pwmB = 33;
int M0_pwmC = 25;
int enable = 12;  //电机使能

//低通滤波初始化
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01);   // Tf = 10ms   //M0速度环
LowPassFilter M0_Curr_Flt = LowPassFilter(0.05);  // Tf = 5ms   //M0电流环
//PID
PIDController vel_loop_M0 = PIDController{2,0,0,100000,voltage_power_supply / 2 };
PIDController angle_loop_M0 = PIDController{2,0, 0,100000,100 };
PIDController current_loop_M0 = PIDController{ 1.2,0,0,100000,12.6 };
//MT6701
Sensor_MT6701 S0 = Sensor_MT6701(0);
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f




//=================PID 设置函数=================
//速度PID
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp, float limit)  //M0角度环PID设置
{
  vel_loop_M0.P = P;
  vel_loop_M0.I = I;
  vel_loop_M0.D = D;
  vel_loop_M0.output_ramp = ramp;
  vel_loop_M0.limit = limit;
}
//角度PID
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit)  //M0角度环PID设置
{
  angle_loop_M0.P = P;
  angle_loop_M0.I = I;
  angle_loop_M0.D = D;
  angle_loop_M0.output_ramp = ramp;
  angle_loop_M0.limit = limit;
}
void DFOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp)  //M0电流环PID设置
{
  current_loop_M0.P = P;
  current_loop_M0.I = I;
  current_loop_M0.D = D;
  current_loop_M0.output_ramp = ramp;
}

//M0速度PID接口
float DFOC_M0_VEL_PID(float error)  //M0速度环
{
  return vel_loop_M0(error);
}
//M0角度PID接口
float DFOC_M0_ANGLE_PID(float error) {
  return angle_loop_M0(error);
}

//初始变量及函数定义
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。1


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);  //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


// 设置PWM到控制器输出
void M0_setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  //写入PWM到PWM 0 1 2 通道
  MCTM0_CH1_SetOnduty( dc_a * 255);
  MCTM0_CH1_SetOnduty( dc_b * 255);
  MCTM0_CH1_SetOnduty( dc_c * 255);
}


void M0_setTorque(float Uq, float angle_el)
{
  if (Uq < 0)
    angle_el += _PI;
  Uq = abs(Uq);

  angle_el = _normalizeAngle(angle_el + _PI_2);
  int sector = floor(angle_el / _PI_3) + 1;
  // calculate the duty cycles
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;

  float Ta, Tb, Tc;
  switch (sector)
  {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;

  M0_setPwm(Ua, Ub, Uc);
}

void DFOC_enable() {
  //digitalWrite(enable, HIGH);
  //引脚至高
}

void DFOC_disable() {
  //digitalWrite(enable, LOW);
  //引脚至低
}

void DFOC_Vbus(float power_supply) {
  voltage_power_supply = power_supply;
  MCTM0_Configuration();
  printf("完成PWM初始化设置\n");
  //pinMode(enable, OUTPUT);  // 设置使能引脚

  //MT6701
  // S0_I2C.begin(19, 18, 400000UL);
  // S0.Sensor_init(&S0_I2C);  //初始化编码器0
  // Serial.println("编码器加载完毕");
  //**********i2c初始化
  //PID 加载
  vel_loop_M0 = PIDController{2,0,0,100000,voltage_power_supply / 2 };

}
float S0_electricalAngle() {
  return _normalizeAngle((float)(M0_DIR * M0_PP) * S0.getMechanicalAngle() - S0_zero_electric_angle);
}



void DFOC_M0_alignSensor(int _PP, int _DIR) {
  M0_PP = _PP;
  M0_DIR = _DIR;
  M0_setTorque(3, _3PI_2);  //起劲
  delay(1000);
  S0.Sensor_update();  //更新角度，方便下面电角度读取
  S0_zero_electric_angle = S0_electricalAngle();
  M0_setTorque(0, _3PI_2);  //松劲（解除校准）
  printf("M0 0电角度：%f\n",S0_zero_electric_angle);
}

float DFOC_M0_Angle() {
  return M0_DIR * S0.getAngle();
}



//=========================电流读取=========================
//有滤波
float DFOC_M0_Velocity() {
  //获取速度数据并滤波
  float vel_M0_ori = S0.getVelocity();
  float vel_M0_flit = M0_Vel_Flt(M0_DIR * vel_M0_ori);
  return vel_M0_flit;  //考虑方向
}


//==============串口接收==============
// float M0_target;
// float M1_target;
// int commaPosition;
// String serialReceiveUserCommand() {

//   // a string to hold incoming data
//   static String received_chars;

//   String command = "";

//   while (Serial.available()) {
//     // get the new byte:
//     char inChar = (char)Serial.read();
//     // add it to the string buffer:
//     received_chars += inChar;

//     // end of user input
//     if (inChar == '\n') {

//       // execute the user command
//       command = received_chars;

//       commaPosition = command.indexOf('\n');  //检测字符串中的逗号
//       if (commaPosition != -1)                //如果有逗号存在就向下执行
//       {
//         M0_target = command.substring(0, commaPosition).toDouble();  //电机角度
//         Serial.println(M0_target);
//       }
//       // reset the command buffer
//       received_chars = "";
//     }
//   }
//   return command;
// }


// float serial_motor_target() {
//   return M0_target;
// }





//================简易接口函数================
void DFOC_M0_setTorque(float Target)  //电流力矩环
{
  //M0_setTorque(current_loop_M0(Target - DFOC_M0_Current()), S0_electricalAngle());
  M0_setTorque(Target,S0_electricalAngle());// dual test
}

void DFOC_M0_set_Velocity_Angle(float Target)  //角度-速度-力 位置闭环
{
  //setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());        //改进前
  DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI) - DFOC_M0_Velocity()));  //改进后
}


void DFOC_M0_setVelocity(float Target)  //速度闭环
{
  //setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //改进前
  DFOC_M0_setTorque(DFOC_M0_VEL_PID((Target - DFOC_M0_Velocity()) * 180 / PI));  //改进后
}

void DFOC_M0_set_Force_Angle(float Target)  //力位闭环
{
  //setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());   //改进前
  DFOC_M0_setTorque(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI));  //改进后
}





void runFOC() {
  //====传感器更新====
  S0.Sensor_update();

  //====传感器更新====
}
