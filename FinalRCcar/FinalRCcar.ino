#include <Servo.h>

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2,3);
Servo sv;


#define ENA 9
#define IN1 4
#define IN2 5

#define ENB 11
#define IN3 6
#define IN4 7


#define FORWARD_IR A0
#define BACKWARD_IR A1
#define LIR A2
#define RIR A3

const int FORWARD = 99;
const int BACKWARD = 98;

char bt;
int for_state;
int speed = 0;
int turnSpeed = 50;
int line_state = 0;

void setup() {
  BTserial.begin(9600);
  Serial.begin(9600);
  //블루투스 통신을 위한 시리얼 모니터 통신 시작

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //L298N 모터 드라이버 OUT 1, OUT 2, Enable A핀 설정

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //L298N 모터 드라이버 OUT 3, OUT 4, Enable B핀 설정

  pinMode(FORWARD_IR, INPUT);
  
  pinMode(LIR, INPUT);
  pinMode(RIR, INPUT);
  speed = 100;
  //sv.attach(8);
}

void loop() {
  for_state = digitalRead(FORWARD_IR);
  int L_state = digitalRead(LIR);
  int R_state = digitalRead(RIR);
  //Serial.println(for_state);
  if(BTserial.available()) //블루투스 통신을 시작하면 아래 코드 실행.
  {
    bt = BTserial.read();
    Serial.println(bt);
    if(bt == 'u')
    {
      forward();
      if(for_state == LOW)
      {
        stop();
      }
    }
    else if(bt == 'd')
    {
      backward();
      if(for_state == LOW)
      {
        stop();
      }
    }
    else if(bt == 'l')
    {
      left();
      if(for_state == LOW)
      {
        stop();
      }
    }
    else if(bt == 'r')
    {
      right();
      if(for_state == LOW)
      {
        stop();
      }
    }
    else if(bt == 'z')
    {
      //sv.write(135);
    }
    else if(bt == 'x')
    {
      //sv.write(45);
    }
    else if(bt == 'c')
    {
      line_state = 1;
      speed - 80;
      linetrace();
    }
    else if(bt == 'v')
    {
      //sv.write(90);
      stop();
    }
  }

  if(for_state == LOW)
  {
    stop();
  }
  
  // 모터 1 제어
  delay(10);
}
void setMotorSpeed(int motorPin1, int motorPin2, int speed, int enable, int direction) {
  // 모터 핀1, 핀2, 속도, Enable 핀, 방향을 입력으로 사용.
  if (direction == FORWARD) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  analogWrite(enable, speed);
}
void forward()
{
  setMotorSpeed(IN1, IN2, speed, ENA, BACKWARD);
  setMotorSpeed(IN3, IN4, speed, ENB, BACKWARD); 
}

void backward()
{
  setMotorSpeed(IN1, IN2, speed, ENA, FORWARD);
  setMotorSpeed(IN3, IN4, speed, ENB, FORWARD);
}

void left()
{
  setMotorSpeed(IN1, IN2, speed, ENA, BACKWARD);
  setMotorSpeed(IN3, IN4, speed, ENB, FORWARD);
}

void right()
{
  setMotorSpeed(IN1, IN2, speed, ENA, FORWARD);
  setMotorSpeed(IN3, IN4, speed, ENB, BACKWARD);
}

//모든 DC모터 중지
void stop()
{
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

/*
해당 적외선 센서는 검은색 절연 테이프를 인식하면 빛이 반사되어 돌아오지 않음.
이것을 이용하여 왼쪽 적외선 센서가 인식되면 우회전 할 수 있도록 구현함.
*/
void linetrace()
{
  while(line_state)
  {
    bt = BTserial.read();
    Serial.println(bt);
    for_state = digitalRead(FORWARD_IR);
    if(digitalRead(LIR) == HIGH && digitalRead(RIR) == HIGH)
    {
      forward();
      if(for_state == LOW)
      {
        stop();
      }
    }
    else if(digitalRead(LIR) == HIGH && digitalRead(RIR) == LOW)
    {
      left(); 
      if(for_state == LOW)
      {
        stop();
      }
      //왼쪽 적외선 센서가 LOW인 것은 라인을 벗어남을 뜻함.
    }
    else if(digitalRead(LIR) == LOW && digitalRead(RIR) == HIGH)
    {
      right();
      if(for_state == LOW)
      {
        stop();
      }
    }
    else if(digitalRead(LIR) == LOW && digitalRead(RIR) == LOW)
    {
      stop();
    }
    
    if(bt == 'v')
    {
      stop();
      line_state = 0;
      speed = 100;
      break;
    }
  }
}