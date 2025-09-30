#include <Wire.h> // 必须包含I2C库
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

// --- 新增：用于非阻塞串口读取的变量 ---
const byte serialBufferSize = 64;   // 定义缓冲区大小，64字节足够了
char serialBuffer[serialBufferSize]; // 缓冲区数组
byte bufferIndex = 0;               // 当前写入到缓冲区的位置

// 新增一个标志位，告诉loop()有新的、解析好的指令可用
volatile bool newTargetAvailable = false; // volatile关键字建议用在主程序和中断/后台处理共用的变量上，更安全

// 我们需要两组目标变量：
// 1. 电机当前正在移动的目标
float current_target0 = 0.0;
float current_target1 = 0.0;
float current_target2 = 0.0;

// 2. 从串口刚刚接收到的新目标 (作为临时中转)
float incoming_target0 = 0.0;
float incoming_target1 = 0.0;
float incoming_target2 = 0.0;

Adafruit_MotorShield AFMS0(0x61);
Adafruit_MotorShield AFMS1(0x60);

Adafruit_StepperMotor *myStepper1 = AFMS0.getStepper(200, 1);
Adafruit_StepperMotor *myStepper2 = AFMS1.getStepper(200, 1);
Adafruit_StepperMotor *myStepper0 = AFMS0.getStepper(200, 2);

void forwardstep0() {
myStepper0->onestep(FORWARD, DOUBLE);
}
void backwardstep0() {
myStepper0->onestep(BACKWARD, DOUBLE);
}
void forwardstep1() {
myStepper1->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {
myStepper1->onestep(BACKWARD, DOUBLE);
}
void forwardstep2() {
myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {
myStepper2->onestep(BACKWARD, DOUBLE);
}

AccelStepper stepper0(forwardstep0, backwardstep0);
AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

String target_s = "";
char target_ca[100];
char *strings[9];
char *ptr = NULL;
int index = 0;

float target0 = 0.0;
float target1 = 0.0;
float target2 = 0.0;

float move_target0 = 0.0;
float move_target1 = 0.0;
float move_target2 = 0.0;

bool distance = false;
bool reach = false;
bool occupy = false;

void setup() {
  Serial.begin(1000000);

  // --- 新增：发送启动就绪信息 ---
  // 这条信息会被 MATLAB 的第一个 readline(s) 捕获并清除，
  // 从而避免了启动时的超时警告。
  Serial.println("Arduino is ready."); 

  // 2. 初始化I2C总线
  Wire.begin();

  // --- 关键修改：根据官方Q&A提高I2C速度 ---
  // 这行代码将I2C时钟频率从默认的100KHz更改为400KHz
  // 注意：它必须在 Wire.begin() 之后调用
  TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz

  AFMS0.begin();
  AFMS1.begin();

  stepper0.setMaxSpeed(100.0); 
  stepper0.setAcceleration(500.0);
  stepper0.moveTo(0);
  stepper1.setMaxSpeed(100.0); 
  stepper1.setAcceleration(500.0);
  stepper1.moveTo(0);
  stepper2.setMaxSpeed(100.0); 
  stepper2.setAcceleration(500.0);
  stepper2.moveTo(0);
}

void loop() {
  // 1. 在循环开始时，处理串口输入（非阻塞）
  handleSerialInput();

  // 2. 检查是否有新的、可用的目标指令
  if (newTargetAvailable) {
    // 首先，重置标志位
    newTargetAvailable = false;
    
    // 将新指令“提交”为当前目标
    current_target0 = incoming_target0;
    current_target1 = incoming_target1;
    current_target2 = incoming_target2;
    
    // 立即让电机转向新的目标
    stepper0.moveTo(current_target0);
    stepper1.moveTo(current_target1);
    stepper2.moveTo(current_target2);

    // 可选：发送一个确认信息
    Serial.print("Target updated to: ");
    Serial.print(current_target0); Serial.print(", ");
    Serial.print(current_target1); Serial.print(", ");
    Serial.println(current_target2);
  }

  // 3. 不管发生什么，永远以最高频率运行电机
  stepper0.run();
  stepper1.run();
  stepper2.run();
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '+') {
      serialBuffer[bufferIndex] = '\0'; // 添加字符串结束符

      // --- 使用 strtok 和 atof 进行解析 ---
      char *ptr = strtok(serialBuffer, ","); // 第一次分割
      int index = 0;
      float temp_values[3]; // 用一个临时数组存解析出的值

      while (ptr != NULL && index < 3) {
        temp_values[index] = atof(ptr); // 将分割出的字符串转为浮点数
        index++;
        ptr = strtok(NULL, ","); // 继续分割
      }
      
      // 如果成功解析出3个值
      if (index == 3) {
        // --- 这是修改的核心 ---
        // 1. 将解析出的值存入“新指令”临时变量
        incoming_target0 = temp_values[0];
        incoming_target1 = temp_values[1];
        incoming_target2 = temp_values[2];
        
        // 2. 升起标志位，通知 loop()
        newTargetAvailable = true;
      } else {
        // 解析失败的逻辑保持不变
        Serial.print("ERROR: Invalid command format -> ");
        Serial.println(serialBuffer);
      }
      
      bufferIndex = 0; // 重置缓冲区

    } else {
      if (bufferIndex < serialBufferSize - 1) {
        serialBuffer[bufferIndex++] = incomingChar;
      }
    }
  }
}
