/* ============================================================================
 *  MeArm 机械臂 —— 摇杆板(ArmJoystick)遥控版
 *  在太极创客《meArm 篇》change.ino 的基础上改造
 *
 *  【本次核心改动】
 *    原来的「手柄模式」是靠串口发 w / s / a / d / 1 / 2 / 3 / 5 这 8 个字符，
 *    按一次舵机挪一点点；现在改成直接读拓展板 ArmJoystick 上的两个物理摇杆，
 *    连续、实时、带限位地驱动 4 个舵机：
 *
 *        左摇杆 左右(X) → base  底盘旋转   （原 a / d）
 *        左摇杆 上下(Y) → rArm  后臂升降   （原 w / s）
 *        右摇杆 上下(Y) → fArm  前臂升降   （原 5 / 2）
 *        右摇杆 左右(X) → claw  夹爪开合   （原 1 / 3）
 *
 *    「指令模式」完整保留：串口输入 b90 / r45 / f60 / c30 / o / i / m 照旧管用。
 *
 *  【摇杆手感对得上的三个关键】
 *    ① 中位标定：上电读一次中位值，摇杆自身有偏差也不会一直漂
 *    ② 死区 DEADZONE：摇杆松手回不到精确中位，没死区机械臂会自己慢慢爬
 *    ③ 比例调速 + 限位：推得越多转得越快；同时硬夹在 [min,max] 之间保护舵机
 *
 *  ★ 摇杆控制必须"非阻塞"：原 servoCmd() 用 for + delay() 一度一度地转，
 *    放进摇杆模式会把手感拖成一卡一卡，所以这里用 millis() 定时 + 增量写角度。
 * ==========================================================================*/

#include <Servo.h>                //使用servo库
Servo base, fArm, rArm, claw;     //创建4个servo对象

/* ========================================================================
 * 1. 舵机行程极限
 *    const指定该数值为常量,常量数值在程序运行中不能改变
 *    ★ 请按你的机械结构实际能转到的范围收紧，别让舵机顶死
 * ======================================================================*/
const int baseMin = 0;
const int baseMax = 180;
const int rArmMin = 0;
const int rArmMax = 120;
const int fArmMin = 90;
const int fArmMax = 180;
const int clawMin = 0;
const int clawMax = 70;

/* ========================================================================
 * 2. 拓展板 ArmJoystick 摇杆引脚  ★★ 必须和你的板子实际接线一致 ★★
 *    常见 meArm 摇杆拓展板：左右两个摇杆各占 2 路模拟口，集中在 A0~A3。
 *    若推杆没反应，先跑 joy_probe.ino 探测各轴到底接在哪个 A 口，再回来改这里。
 *    注意：A4 / A5 同时是 I2C 的 SDA / SCL，有 I2C 模块在用就别把摇杆放这两路。
 * ======================================================================*/
const int pinJoy_LX = A0;         // 左摇杆 水平 VRx
const int pinJoy_LY = A1;         // 左摇杆 垂直 VRy
const int pinJoy_RX = A2;         // 右摇杆 水平 VRx
const int pinJoy_RY = A3;         // 右摇杆 垂直 VRy

/*  推杆方向与舵机转向相反时，把对应的 1 改成 -1 即可  */
const int DIR_LX = -1;             // 左摇杆左右（推左 → base 想往哪转）
const int DIR_LY = -1;             // 左摇杆上下 // 改
const int DIR_RX = -1;             // 右摇杆左右
const int DIR_RY = -1;             // 右摇杆上下

/* ========================================================================
 * 3. 运行参数
 * ======================================================================*/
#define KEEP_WSAD_FALLBACK 1      // 1 = 保留串口 wsad/1235 单字符手动微调（老蓝牙玩法）
                                  // 0 = 彻底删掉，串口只管指令模式那套 b/r/f/c/o/i/m

int DSD = 30;                     //Default Servo Delay (默认电机运动延迟时间)
                                  //此变量用于控制电机运行速度.增大此变量数值将
                                  //降低电机运行速度从而控制机械臂动作速度。

const bool MODE_CMD = true;       // mode = 1: 指令模式
const bool MODE_JOY = false;      // mode = 0: 手柄(摇杆)模式
bool mode = MODE_JOY;             // 上电默认进摇杆模式（原代码靠未初始化的全局变量默认成 0）

int moveStep = 3;                 // 每一次按下手柄按键，舵机移动量（仅串口单字符微调用）

const int DEADZONE = 60;          // 摇杆死区（±60/1023，约 ±6%）
const int MAX_STEP = 1;           // 每 20ms 最多转多少度 → 最高约 200°/s  // 改
const int UPDATE_MS = 20;         // 摇杆刷新周期(ms)
const int AXIS_SAMPLES = 4;       // 模拟量多次采样取平均，抗抖动

/* ========================================================================
 * 4. 摇杆中位（开机自动标定）
 * ======================================================================*/
int cLX, cLY, cRX, cRY;
unsigned long lastJoyMs = 0;      // 摇杆刷新计时

/* ========================================================================
 * 5. 函数原型
 *    Arduino IDE 会自动生成，这里显式写出便于阅读，
 *    也方便用命令行工具链（avr-g++）直接编译
 * ======================================================================*/
void armDataCmd(char serialCmd);
void armJoyCmd(char serialCmd);
void servoCmd(char servoName, int toPos, int servoDelay);
void reportStatus();
void armIniPos();
int  readAxisAvg(int pin);
void calibrateJoystick();
int  axisToStep(int joyPin, int center, int dir);
void nudgeServo(Servo &s, int step, int minA, int maxA);
void updateJoystick();
void reportJoyStick();

/* ========================================================================== */
void setup(){
  base.attach(9);     // base 伺服舵机连接引脚9 舵机代号'b'
  delay(200);          // 稳定性等待
  rArm.attach(8);     // rArm 伺服舵机连接引脚8 舵机代号'r'(左侧)
  delay(200);          // 稳定性等待
  fArm.attach(7);      // fArm 伺服舵机连接引脚7  舵机代号'f'(右侧)
  delay(200);          // 稳定性等待
  claw.attach(6);      // claw 伺服舵机连接引脚6 舵机代号'c'
  delay(200);          // 稳定性等待

  base.write(90);
  delay(10);
  fArm.write(90);
  delay(10);
  rArm.write(90);
  delay(10);
  claw.write(0);
  delay(10); // 设置初始化

  Serial.begin(9600);
  Serial.println("Welcome to Taichi-Maker Robot Arm Tutorial");
  Serial.println("Mode: JOY-STICK (ArmJoystick 双摇杆)  --  serial 'm'/'i'/'o' still works");

  delay(300);              // 给摇杆一点回中时间
  calibrateJoystick();     // ★上电标定中位，此刻不要去碰摇杆
}

/* ========================================================================== */
void loop(){
  /* ---- 串口：指令模式 / 老式单字符微调，原功能保留 ---- */
  while (Serial.available() > 0) {
    char serialCmd = Serial.read();
    if (mode == MODE_CMD) {
      armDataCmd(serialCmd); //指令模式 对应一个函数
    } else {
      armJoyCmd(serialCmd);  //手柄模式 对应另一个函数
    }
  }

  /* ---- 摇杆：只有摇杆模式才实时读取（指令模式下摇杆不干预） ---- */
  if (mode == MODE_JOY) {
    updateJoystick();
  }
}

/* ==========================================================================
 *  摇杆读取相关
 * ==========================================================================*/

/*  多次采样取平均，抑制抖动  */
int readAxisAvg(int pin){
  long sum = 0;
  for (int i = 0; i < AXIS_SAMPLES; i++) sum += analogRead(pin);
  return (int)(sum / AXIS_SAMPLES);
}

/*  标定中位：以"摇杆静止时的读数"作为中位参考  */
void calibrateJoystick(){
  cLX = readAxisAvg(pinJoy_LX);
  cLY = readAxisAvg(pinJoy_LY);
  cRX = readAxisAvg(pinJoy_RX);
  cRY = readAxisAvg(pinJoy_RY);

  Serial.print("+JoyStick center (L_X, L_Y, R_X, R_Y): ");
  Serial.print(cLX); Serial.print(", ");
  Serial.print(cLY); Serial.print(", ");
  Serial.print(cRX); Serial.print(", ");
  Serial.println(cRY);
  Serial.println("  (正常应都在 480~540 附近；若某一路是 0 或 1023，说明那条线没接好)");
}

/*  把一路模拟量换算成"本周期该转多少度"
 *  返回值 -MAX_STEP ~ +MAX_STEP，0 表示在死区内不动           */
int axisToStep(int joyPin, int center, int dir){
  int d = readAxisAvg(joyPin) - center;
  if (abs(d) <= DEADZONE) return 0;            // 死区 → 不动

  // 死区之外还剩多少行程，用于把"推杆幅度"线性映射成"速度"
  int span = (d > 0) ? (1023 - center - DEADZONE)
                     : (center - DEADZONE);
  if (span < 1) span = 1;

  int mag  = abs(d) - DEADZONE;                // 0 .. span
  int step = (int)((long)mag * MAX_STEP / span); // 0 .. MAX_STEP
  if (step < 1) step = 1;                      // 一出死区至少动 1 度
  if (step > MAX_STEP) step = MAX_STEP;

  return (d > 0 ? step : -step) * dir;
}

/*  按增量移动舵机，并夹在行程极限内（不阻塞，立刻返回）  */
void nudgeServo(Servo &s, int step, int minA, int maxA){
  if (step == 0) return;
  int cur = s.read();            // Servo.read() 返回上一次写入的角度
  int to  = cur + step;
  if (to < minA) to = minA;
  if (to > maxA) to = maxA;
  if (to != cur) s.write(to);
}

/*  摇杆主循环：定时刷新，非阻塞
 *  轴 → 舵机 映射想互换，只要把下面两行的 pinJoy_RX / pinJoy_RY 换个位置即可  */
void updateJoystick(){
  if (millis() - lastJoyMs < UPDATE_MS) return;
  lastJoyMs = millis();

  nudgeServo(base, axisToStep(pinJoy_LX, cLX, DIR_LX), baseMin, baseMax); // 左摇杆左右 → 底盘（原 a/d）
  nudgeServo(rArm, axisToStep(pinJoy_LY, cLY, DIR_LY), rArmMin, rArmMax); // 左摇杆上下 → 后臂（原 w/s）
  nudgeServo(fArm, axisToStep(pinJoy_RY, cRY, DIR_RY), fArmMin, fArmMax); // 右摇杆上下 → 前臂（原 5/2）
  nudgeServo(claw, axisToStep(pinJoy_RX, cRX, DIR_RX), clawMin, clawMax); // 右摇杆左右 → 夹爪（原 1/3）
}

/*  摇杆原始值打印，接线/调机时很有用（输入 j）  */
void reportJoyStick(){
  Serial.println("");
  Serial.println("+ Joy-Stick Raw Values +");
  Serial.print("L_X(A0): "); Serial.print(readAxisAvg(pinJoy_LX)); Serial.print("  center "); Serial.println(cLX);
  Serial.print("L_Y(A1): "); Serial.print(readAxisAvg(pinJoy_LY)); Serial.print("  center "); Serial.println(cLY);
  Serial.print("R_X(A2): "); Serial.print(readAxisAvg(pinJoy_RX)); Serial.print("  center "); Serial.println(cRX);
  Serial.print("R_Y(A3): "); Serial.print(readAxisAvg(pinJoy_RY)); Serial.print("  center "); Serial.println(cRY);
  Serial.println("++++++++++++++++++++++++");
  Serial.println("");
}

/* ==========================================================================
 *  指令模式
 * ==========================================================================*/
void armDataCmd(char serialCmd){ //Arduino根据串行指令执行相应操作
                                 //指令示例：b45 底盘转到45度角度位置
                                 //          o 输出机械臂舵机状态信息
  //判断人类是否因搞错模式而输入错误的指令信息（指令模式下输入手柄按键信息）
  //★ 原代码这里写成了 '8','4','6','8'（重复且漏了 1/2/3/5），属笔误，已修正
  if (   serialCmd == 'w' || serialCmd == 's' || serialCmd == 'a' || serialCmd == 'd'
      || serialCmd == '1' || serialCmd == '2' || serialCmd == '3' || serialCmd == '5' ){
    Serial.println("+Warning: Robot in Instruction Mode...");
    delay(100);
    while(Serial.available()>0) Serial.read();  //清除串口缓存的错误指令
    return;
  } // 如果在mode = 0时输入手柄的消息 则不走这条函数


  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r'){
    int servoData = Serial.parseInt();// 读取后面的数据
    servoCmd(serialCmd, servoData, DSD);  // 机械臂舵机运行函数（参数：舵机名，目标角度，延迟/速度）
  } else {
    switch(serialCmd){

      case 'm' :   //切换至手柄模式
        mode = MODE_JOY;
        Serial.println("Command: Switch to Joy-Stick Mode.");
        break;

      case 'o':  // 输出舵机状态信息
        reportStatus();
        break;

      case 'i':
        armIniPos();
        break;

      case 'j':  // ★新增：查看摇杆原始值（调机用）
        reportJoyStick();
        break;

      case 'k':  // ★新增：重新标定摇杆中位（把手松开再发 k）
        Serial.println("+Recalibrate: keep both sticks at rest...");
        delay(300);
        calibrateJoystick();
        break;

      default:  //未知指令反馈
        Serial.println("Unknown Command.");
    }
  }
}

/* ==========================================================================
 *  手柄(摇杆)模式
 *  真正的动作由物理摇杆在 updateJoystick() 里持续驱动；
 *  这里只处理串口来的辅助指令，外加可选的老式单字符微调。
 * ==========================================================================*/
void armJoyCmd(char serialCmd){

  //判断人类是否因搞错模式而输入错误的指令信息（手柄模式下输入舵机指令）
  if (serialCmd == 'b' || serialCmd == 'c' || serialCmd == 'f' || serialCmd == 'r'){
    Serial.println("+Warning: Robot in Joy-Stick Mode...");
    delay(100);
    while(Serial.available()>0) Serial.read();  //清除串口缓存的错误指令
    return;
  }

#if KEEP_WSAD_FALLBACK
  int baseJoyPos;
  int rArmJoyPos;
  int fArmJoyPos;
  int clawJoyPos;
#endif

  switch(serialCmd){

    /* ---- ↓↓↓ 以下 8 个 case = 原来的 wsad / 1235，现在只是可选的手动微调 ↓↓↓ ---- */
#if KEEP_WSAD_FALLBACK
    case 'a':  // Base向左
      Serial.println("Received Command: Base Turn Left");
      baseJoyPos = base.read() + moveStep;// 向左移动moveStep个单位
      servoCmd('b', baseJoyPos, DSD);
      break;

    case 'd':  // Base向右
      Serial.println("Received Command: Base Turn Right");
      baseJoyPos = base.read() - moveStep;// 向右移动moveStep个单位
      servoCmd('b', baseJoyPos, DSD);
      break;

    case 's':  // rArm向下
      Serial.println("Received Command: Rear Arm Down");
      rArmJoyPos = rArm.read() + moveStep;// 同理
      servoCmd('r', rArmJoyPos, DSD);
      break;

    case 'w':  // rArm向上
      Serial.println("Received Command: Rear Arm Up");
      rArmJoyPos = rArm.read() - moveStep;// 同理
      servoCmd('r', rArmJoyPos, DSD);
      break;

    case '5':  // fArm向上
      Serial.println("Received Command: Front Arm Up");
      fArmJoyPos = fArm.read() + moveStep;
      servoCmd('f', fArmJoyPos, DSD);
      break;

    case '2':  // fArm向下
      Serial.println("Received Command: Front Arm Down");
      fArmJoyPos = fArm.read() - moveStep;
      servoCmd('f', fArmJoyPos, DSD);
      break;

    case '1':  // Claw关闭
      Serial.println("Received Command: Claw Close Down");
      clawJoyPos = claw.read() + moveStep;
      servoCmd('c', clawJoyPos, DSD);
      break;

    case '3':  // Claw打开
      Serial.println("Received Command: Claw Open Up");
      clawJoyPos = claw.read() - moveStep;
      servoCmd('c', clawJoyPos, DSD);
      break;
#endif
    /* ---- ↑↑↑ 可选微调结束；把 KEEP_WSAD_FALLBACK 改成 0 即整段消失 ↑↑↑ ---- */

    case 'm' :   //切换至指令模式
      mode = MODE_CMD;
      Serial.println("Command: Switch to Instruction Mode.");
      break;

    case 'o':
      reportStatus();// 显示状态
      break;

    case 'i':
      armIniPos();// 复原
      break;

    case 'j':    // ★新增：查看摇杆原始值
      reportJoyStick();
      break;

    case 'k':    // ★新增：重新标定摇杆中位
      Serial.println("+Recalibrate: keep both sticks at rest...");
      delay(300);
      calibrateJoystick();
      break;

    default:
      Serial.println("Unknown Command.");
      return;

  }
}

/*  舵机平滑运动（原样保留，用于指令模式 / 回原点）
 *  ★ 注意：这个函数内部是 for + delay()，会阻塞，不要放进摇杆的实时刷新里 */
void servoCmd(char servoName, int toPos, int servoDelay){
  Servo servo2go;  //创建servo对象

  //串口监视器输出接收指令信息
  Serial.println("");
  Serial.print("+Command: Servo ");
  Serial.print(servoName);
  Serial.print(" to ");
  Serial.print(toPos);
  Serial.print(" at servoDelay value ");
  Serial.print(servoDelay);
  Serial.println(".");
  Serial.println("");

  int fromPos = 0; //建立变量，存储电机起始运动角度值

  switch(servoName){
    case 'b':
      if(toPos >= baseMin && toPos <= baseMax){
        servo2go = base;
        fromPos = base.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: Base Servo Value Out Of Limit!");
        return;
      }

    case 'c':
      if(toPos >= clawMin && toPos <= clawMax){
        servo2go = claw;
        fromPos = claw.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: Claw Servo Value Out Of Limit!");
        return;
      }

    case 'f':
      if(toPos >= fArmMin && toPos <= fArmMax){
        servo2go = fArm;
        fromPos = fArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: fArm Servo Value Out Of Limit!");
        return;
      }

    case 'r':
      if(toPos >= rArmMin && toPos <= rArmMax){
        servo2go = rArm;
        fromPos = rArm.read();  // 获取当前电机角度值用于“电机运动起始角度值”
        break;
      } else {
        Serial.println("+Warning: rArm Servo Value Out Of Limit!");
        return;
      }

    default:  //★ 舵机名不认识就直接退出，避免拿未初始化的 fromPos 去跑一个几万次的阻塞循环
      Serial.println("+Warning: Unknown Servo Name!");
      return;
  }

  //指挥电机运行
  if (fromPos <= toPos){  //如果“起始角度值”小于“目标角度值”
    for (int i=fromPos; i<=toPos; i++){
      servo2go.write(i);
      delay (servoDelay);
    }
  }  else { //否则“起始角度值”大于“目标角度值”
    for (int i=fromPos; i>=toPos; i--){
      servo2go.write(i);
      delay (servoDelay);
    }
  }
}

void reportStatus(){  //舵机状态信息
  Serial.println("");
  Serial.println("");
  Serial.println("+ Robot-Arm Status Report +");
  Serial.print("Claw Position: "); Serial.println(claw.read());
  Serial.print("Base Position: "); Serial.println(base.read());
  Serial.print("Rear  Arm Position:"); Serial.println(rArm.read());
  Serial.print("Front Arm Position:"); Serial.println(fArm.read());
  Serial.print("Mode: "); Serial.println(mode == MODE_CMD ? "Instruction" : "Joy-Stick");
  Serial.println("++++++++++++++++++++++++++");
  Serial.println("");
}

// 复原函数
void armIniPos(){
  Serial.println("+Command: Restore Initial Position.");
  int robotIniPosArray[4][3] = {
    {'b', 90, DSD},
    {'r', 90, DSD},
    {'f', 90, DSD},
    {'c', 0, DSD}
  };

  for (int i = 0; i < 4; i++){
    servoCmd(robotIniPosArray[i][0], robotIniPosArray[i][1], robotIniPosArray[i][2]);
  }
}
