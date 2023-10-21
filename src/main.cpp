/*
   ====小车器件清单==================================
   控制器：esp32
   马达：GA25-370减速电机 ，转速大约280rmp。
   马达驱动模块：DRV8833
   陀螺仪模块：MPU6050
   ====马达测试参数 6v电压供给下=======================
   pwm+为前进，pwm+-为后退，均测试
   左马达IN1 IN2, OUT1 OUT2 , PWM12起转。（mpu6050接口侧）
   右马达IN3 IN4，OUT3 OUT4 , PWM12起转。
   ====陀螺仪测试参数================================
   小车静态机械平衡时的角度为-1.6度。
   ====PID调试结果==================================
  P  I   D  调试结果1  P:15      I:0.5       D:0.6
  P  I   D  调试结果2  P:        I:          D:
  ------------------------------------------------------
*/

#include "BluetoothSerial.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define IN1 12
#define IN2 14
#define IN3 26
#define IN4 27

int sda_pin = 4, scl_pin = 2; //自定义eps32的I2C引脚

TwoWire mpuWire = TwoWire(1);

/*---调试和预设定值的变量--------*/
float Balance_Angle_raw = -1.6;    //测试出的静态机械平衡角度。
int leftMotorPwmOffset = 12, rightMotorPwmOffset = 12; //测试出的左右轮的启动pwm值，pwm达到一定电压马达才开始转动。
float ENERGY = 4;                    //设定的前进后退倾倒角度幅度（越大，前冲后退越快）
float turn_ENERGY = 600;               //设定的转向幅度，转向的角速度幅度。
float kp = 15, ki = 0.5, kd = 0.6;     //根据调试测试得到设置kp ki kd的值
int count = 0;
/*---调试和预设定的变量 end--------*/

/*---调试和控制变量--------*/
float Keep_Angle, bias, integrate;              //保持角度，角度偏差，偏差积分变量
float AngleX, GyroX, GyroZ;                     // mpu6050输出的角度值为浮点数，两位有效小数
int vertical_PWM, turn_PWM, PWM, L_PWM, R_PWM;  //各种PWM计算值
float turn_kp = 0.7, turn_spd;                        //转向pwm比例向，转向速度设定
/*---调试和控制变量 end--------*/

char flag; //预留标签

Adafruit_SSD1306 display(128,32,&Wire,-1); //屏幕

BluetoothSerial SerialBT; //实例化esp32的蓝牙串口对象
MPU6050 mpu6050(mpuWire);    //实例化mpu6050对象

void oledDlay(float x, float y, float z, String s) {
  display.clearDisplay();

  display.setCursor(0,0);
  display.print("aglX: ");
  display.println(x);
  display.print("gyrX: ");
  display.println(y);
  display.print("gyrZ: ");
  display.println(z);
  display.print("Bluetooth status: ");
  display.println(s);

  display.setCursor(67,0);
  display.print("  make by");
  display.setCursor(64,8);
  display.print("   Chance");
  display.setCursor(64,16);
  display.print("   :)");
  
  display.display();
}

void motor(int left_EN, int right_EN)   //马达输出函数
{
  left_EN = constrain(left_EN, -255, 255);
  right_EN = constrain(right_EN, -255, 255);  //限定PWM区间在-255~255
  if (left_EN >= 0)
  {
    analogWrite(IN1, left_EN);
    analogWrite(IN2, 0);
  }
  if (left_EN < 0)
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0 - left_EN);
  }
  if (right_EN >= 0)
  {
    analogWrite(IN3, right_EN);
    analogWrite(IN4, 0);
  }
  if (right_EN < 0)
  {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0 - right_EN);
  }
}

void serial_debug()   //蓝牙串口调试和控制函数，根据手机端发送来的串口数据调试或控制
{
  if (SerialBT.available() > 0)
  {
    char DATA = SerialBT.read();
    delay(5);
    switch (DATA)
    {
      /*机械平衡点调整*/
      case 'u':
        Keep_Angle += 0.1;
        break;
      case 'd':
        Keep_Angle -= 0.1;
        break; //调节物理平衡点,a前倾，b后仰

      /*直立环调试*/
      case '0':
        kp -= 1; //调节直立环 比例kp项-
        break;
      case '1':
        kp += 1; //调节直立环 比例kp项+
        break;
      case '2':
        ki -= 0.1; //调节直立环 积分项ki-
        break;
      case '3':
        ki += 0.1; //调节直立环 积分项ki+
        break;
      case '4':
        kd -= 0.1; //调节直立环 微分项kd-
        break;
      case '5':
        kd += 0.1; //调节直立环 微分项kd+
        break;

      /*转向环调试（只用到了比例项）*/
      case '6':
        turn_kp -= 0.2;  //调节转向环 比例项-
        break;
      case '7':
        turn_kp += 0.2;  //调节转向环 比例项+
        break;

      /*蓝牙串口控制程序*/
      case 's': //停车
        Keep_Angle = Balance_Angle_raw;  //调节物理平衡点为机械平衡角度值，原地平衡
        break;
      case 'a': //前进
        Keep_Angle = Balance_Angle_raw + ENERGY; //通过设定需保持的倾角，使得小车前进后退
        break;
      case 'b': //后退
        Keep_Angle = Balance_Angle_raw - ENERGY;
        break;
      case 'z': //不转向
        flag = 'z';
        turn_spd = 0; //设定Z轴目标角速度为0，即是不转向
        break;
      case 'l': //左转
        flag = 'l';
        turn_spd = turn_ENERGY; //顺时针旋转，Z轴目标角速度为正
        break;
      case 'r': //右转
        flag = 'r';
        turn_spd = -turn_ENERGY;  //逆时针旋转，Z轴目标角速度为负
        break;
    }

    /*调试时PID极性限制*/
    if (kp < 0)kp = 0;
    if (ki < 0)ki = 0;
    if (kd < 0)kd = 0;

    /*串口打印输出显示*/
    SerialBT.print("Keep_Angle: ");
    SerialBT.println(Keep_Angle);
    SerialBT.print("kp:");
    SerialBT.print(kp);
    SerialBT.print("  ki:");
    SerialBT.print(ki);
    SerialBT.print("  kd:");
    SerialBT.println(kd);
    SerialBT.print("  turn_kp:");
    SerialBT.println(turn_kp);
    SerialBT.println("--------------------");
  }
}

void vertical_pwm_calculation() //直立PMW计算
{
  AngleX = mpu6050.getAngleX();
  GyroX = mpu6050.getGyroX();
  bias = AngleX - Keep_Angle; // 计算角度偏差。bias为小车角度是静态平衡角度的差值。
  integrate += bias; //偏差的积分，integrate为全局变量，一直积累。
  integrate = constrain(integrate, -1000, 1000); //限定误差积分的最大和最小值
  /*==直立PID计算PWM==通过陀螺仪返回数据计算，前倾陀螺仪Y轴为正，后仰陀螺仪Y轴为负。
    前倾车前进，后仰车后退，保持直立。但可能为了直立，车会随时移动。*/
  vertical_PWM = kp * bias + ki * integrate + kd * GyroX;
}

void turn_pwm_calculation()  //转向PMW计算
{
  GyroZ = mpu6050.getGyroZ(); //获取陀螺仪Z轴角速度
  turn_PWM = turn_kp * (turn_spd - GyroZ);
  //  turn_PWM = constrain(turn_PWM, -180, 180);
}

void motor_control() //马达PWM控制函数
{
  /*---【补偿右轮pwm差值】------------*/
  if (PWM >= 0)
  {
    L_PWM = PWM + leftMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
    R_PWM = PWM + rightMotorPwmOffset; //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (PWM < 0)
  {
    L_PWM = PWM - leftMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
    R_PWM = PWM - rightMotorPwmOffset; //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (PWM == 0)
  {
    L_PWM = 0; //
    R_PWM = 0; //
  }

  /*---【转向控制 左右轮pwm修正值】-------------*/
  L_PWM -= turn_PWM;
  R_PWM += turn_PWM; //控制两轮的pwm差值，通过左轮加速，右轮减速。使其右转

  /*---【控制马达输出】-------------*/
  L_PWM = constrain(L_PWM, -255, 255); //计算出来的PWM限定大小。255为输出上限。
  R_PWM = constrain(R_PWM, -255, 255);

  motor(L_PWM, R_PWM);

  /*--------判断是否小车倒下，此时停止马达和编码器计数-----*/
  if (AngleX > 45 || AngleX < -45) //倾角过大（车倒下时），停止马达输出
  {
    motor(0, 0);
  }
}


void setup() {  //初始化
  display.begin();
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("     Please wait");
  display.println("  ");
  display.println("   INITIALIZING...");
  display.display();

  SerialBT.begin("ESP32_car"); // Bluetooth device name
  mpuWire.begin(sda_pin, scl_pin);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
  //  mpu6050.setGyroOffsets(*, *, *);
  Keep_Angle = Balance_Angle_raw; //平衡角度初始化为静态平衡时的陀螺仪角度。Keep_Angle可以改变，才可以控制前进后退。
  motor(0, 0);                    //机器启动时马达确保停止。
 
  delay(10);                      //循环前延时，确保各种初始和准备完成
}

void loop() {
  /*====串口PID调试+控制===*/
  serial_debug();

  /*====陀螺仪刷新===*/
  mpu6050.update();

  /*====PWM计算====*/
  vertical_pwm_calculation(); //直立环PWM计算

  turn_pwm_calculation(); //转向PWM计算

  PWM = vertical_PWM;

  /*====马达输出=====*/
  motor_control();

  count++;
 
 if (count >= 500)
 {
  oledDlay(AngleX,GyroX,GyroZ,"on");
  count = 0;
 }
 
  
}