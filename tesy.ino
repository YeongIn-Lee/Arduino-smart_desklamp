#include <Adafruit_NeoPixel.h>    //LED
#include <Arduino_FreeRTOS.h>   //멀티태스킹
#include <pitches.h>    //부저
#include <LiquidCrystal.h>    //text LCD

// 핀 번호 (RS, E, DB4, DB5, DB6, DB7)
LiquidCrystal lcd(44, 45, 46, 47, 48, 49); // LCD 연결  *확정*

//블루투스 통신
#define __MEGA2560__ 1          //0: Uno, 1: Mega 2560 

#if __MEGA2560__
#define RxD 18
#define TxD 19
#define BTserial Serial1      //the software serial port
#else
#include <SoftwareSerial.h>       //Software Serial Port
#define RxD 2
#define TxD 3
SoftwareSerial BTserial(RxD, TxD); //the software serial port
#endif

#define BAUDRATE 9600

#ifdef _AVR_
#include <avr/power.h>
#endif

#define PIN 8                     // 디지털핀 어디에 연결했는지 입력   *확정*
#define LEDNUM 20

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDNUM, PIN, NEO_GRBW + NEO_KHZ800);

String myString = "";

int brightLevel=3; //밝기조절단계를 3에서 시작
int BRIGHTNESS;    //led 밝기값

//변수들1
int echoPin = 10;   //초음파센서 input 연결핀
int trigPin = 9;   //초음파센서 output 연결핀
int speakerPin = A3;   //부저 연결핀
int melody[] = {NOTE_C6, NOTE_C5};    //부저 출력음
int selectMode = 0;     //모드변경변수
int modeButtonPin = 3;    //모드변경버튼 연결핀(절전, 졸음방지)   *확정*
int modeButtonCount = 0;     //모드변경버튼 누른횟수(확실한 버튼입력을 위해 필요)
boolean modeStatePrevious = false;
boolean modeStateCurrent;      //모드변경버튼상태 체크
int ledPin = 8;      //LED 연결핀    *확정*
int pirPin = 7;     //PIR센서 연결핀
int pirPreState = LOW;     //PIR센서 초기상태는 움직임이 없음을 가정
int pirCurState;     //PIR센서 현재상태 확인 변수
int timeCount = 1;      //시간 카운트다운 변수(불을키고 난뒤의 시간, 움직임감지모드의 경우에는 감지가 안되는 시간)
int illuminance = A1;   //조도센서 연결핀
int environment = 3;
int powerPin = 2;      //전원버튼핀   *확정*
int powerState = 0;      //전원on/off
int powerCount = 0;     //전원버튼 누른횟수
//boolean powerStatePrevious = false;
//boolean powerStateCurrent;      //전원버튼상태 체크
unsigned long time_current, time_previous;
unsigned long time_previous1, time_current1;
unsigned long time_current2, time_previous2;    //millis 사용을 위한 변수들

//변수들2
byte patterns[] = {
  0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE4, 0xFE, 0xE6
};   //부저 출력음 패턴
int digit_select_pin[] = {66, 67, 68, 69}; // 자릿수 선택 핀
// 7세그먼트 모듈 연결 핀 ‘a, b, c, d, e, f, g, dp’ 순서   *확정*
int segment_pin[] = {58, 59, 60, 61, 62, 63, 64, 65};     //  *확정*
long SEGMENT_DELAY = 3;
int minutes = 0, seconds = 0, hours = 0, prev_seconds = 0; //시간
int button_pin1 = 4;   //타이머, 알람 모드 설정   *확정*
int button_pin2 = 5;   //타이머 초기화, 알람 시간설정   *확정*
int button_pin3 = 6;   //타이머 실행/정지   *확정*
boolean state_previous = false, state_previous2 = false, state_previous3 = false; //버튼 상태
boolean state_current, state_current2, state_current3;
boolean state_button = true, state_button3 = true, state_button_prev;
int state_button2 = 1;

//segment사용을 위한 함수들
void show_digit(int pos, int number) { // (위치, 출력할 숫자)
  for (int i = 0; i < 4; i++) {
    if (i + 1 == pos) // 해당 자릿수의 선택 핀만 LOW로 설정
      digitalWrite(digit_select_pin[i], LOW);
    else // 나머지 자리는 HIGH로 설정
      digitalWrite(digit_select_pin[i], HIGH);
  }
  for (int i = 0; i < 8; i++) {
    boolean on_off = bitRead(patterns[number], 7 - i);
    digitalWrite(segment_pin[i], on_off);
    if (pos == 2) {
      digitalWrite(segment_pin[7], HIGH);
    }
  }
}

void show_4_digit(int number) {
  number = number % 10000;
  int thousands = number / 1000;
  number = number % 1000;
  int hundreads = number / 100;
  number = number % 100;
  int tens = number / 10;
  int ones = number % 10;
  long budget = SEGMENT_DELAY;

  show_digit(1, thousands);
  delay(SEGMENT_DELAY);
  show_digit(2, hundreads);
  delay(SEGMENT_DELAY);
  show_digit(3, tens);
  delay(SEGMENT_DELAY);
  show_digit(4, ones);
}

//버튼눌림확인 함수
boolean button(int buttonPin) {
  static boolean powerStatePrevious[5] = {false, false, false, false, false};
  static boolean powerStateCurrent[5];
  int index = buttonPin - 2;
  boolean result = false;
  powerStateCurrent[index] = digitalRead(buttonPin);
  if (powerStateCurrent[index]) {
      if (powerStatePrevious[index] == false) {
        powerStatePrevious[index] = true;
        result = true;
      }
      vTaskDelay( 50 / portTICK_PERIOD_MS );   //속도저하 문제 발생가능있음.
    }
    else {
      powerStatePrevious[index] = false;
    }
    return result;
}

//주변환경의 단계 받아오는 함수
int environmentState(int trigPin, int echoPin, int illuminancePin) {
  float duration, distance;
      digitalWrite(trigPin, LOW);
      digitalWrite(echoPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      vTaskDelay( 10 / portTICK_PERIOD_MS );
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = ((float)(340 * duration) / 10000) / 2;
      int distance_value = map(distance, 0, 50, 1, 5);    //거리 맵핑

      int senVal = analogRead(illuminancePin);         //조도센서 입력
      int bright_value = map(senVal, 200, 1000, 1, 5);    //주변 밝기 맵핑, 어두울수록 값커짐

      int average = distance_value * bright_value / 2;    //밝기 추천을 위해 현재상태를 1부터5로 나타내어 거기에맞는 밝기를 추천해줌
      //Serial.println(average);    //테스트

      return average;
}

void ledOn(){
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.setPixelColor(0, 255, 255, 255, 255);
  strip.setPixelColor(1, 255, 255, 255, 255);
  strip.setPixelColor(2, 255, 255, 255, 255);
  strip.setPixelColor(3, 255, 255, 255, 255);
  strip.setPixelColor(4, 255, 255, 255, 255);
  strip.setPixelColor(5, 255, 255, 255, 255);
  strip.setPixelColor(6,255,255,255,0);
  strip.show();
}

void ledOff(){
  strip.setBrightness(0);
  strip.begin();
  strip.show();
}

void dontSleep() {
  for (int i = 0; i < 5; i++) {
    ledOff();
    tone(speakerPin, melody[0], 500);
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    noTone(speakerPin);
    ledOn();
    tone(speakerPin, melody[1], 500);
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    noTone(speakerPin);
    //부저음,LED깜박임
    }
    modeButtonCount = 0;     //모드를 다시 절전모드로 변경
    timeCount = 1;
}

void powerSaving() {
  ledOff();
  powerCount++;
}

long get_number_from_bluetooth()
{
  static const int MAX_BUFFER_SIZE = 256;
  static char buffer[MAX_BUFFER_SIZE];
  long result = -1;
  int idx = 0;
  char ch;

  buffer[0] = 0;
  do {
    if (BTserial.available()) {
      if (idx >= MAX_BUFFER_SIZE) {
        Serial.print("Buffer overflow detected... return -1;");
        return -1;
      }
      ch = BTserial.read();
      buffer[idx] = ch;
      idx = idx + 1;
    }
    // vTaskDelay( 50 / portTICK_PERIOD_MS );   //속도저하 문제 발생가능있음
  } while (ch != '\n');
  buffer[idx - 1] = 0;
  result = atol(buffer);
  return result;
}

//멀티태스킹
void Task1( void *pvParameters );
void Task2( void *pvParameters );
void Task3( void *pvParameters );

void setup() {

  /* 9600 baudrate로 시리얼 통신을 진행하도록 합니다. */
  Serial.begin(9600);

  Serial.begin(BAUDRATE);
  BTserial.begin(BAUDRATE);

  strip.begin();
  strip.show();

  while (!Serial) {
    ; /* (선택적) 시리얼이 준비가 되지 않는 경우가 있을 수 있기 때문에, 준비를 시켜주도록 합니다. */
  }

  /*
     독립적으로 돌아갈 2개의 Task를 생성하도록 합니다.
  */
  xTaskCreate(
    Task1    /* 독립적으로 실행 시킬 함수에 해당합니다. */
    ,  "segment관련"   /* 이게 어떤 Task인지 설명하는 부분으로 주석과 같은 역할이라고 보면 됩니다. 아무 문자열을 넣어도 상관 없습니다. */
    ,  128       /* 이 Task에 할당이 될 스택의 크기입니다. CPU의 WORD 바이트 값을 곱해주면 실제 크기를 계산할 수 있습니다. */
    ,  NULL      /* 함수의 pvParameters로 넘길 인수에 해당합니다. 별 일 없으면 NULL로 설정하면 됩니다. */
    ,  1        /* 이 Task의 우선 순위에 해당합니다. 우선 순위가 높을수록 먼저 실행됩니다. (0(가장 작음) ~ 3(가장 큼)) */
    ,  NULL      /* 이 Task를 다른 곳에서 다루는 핸들을 설정하는 부분입니다. 별 일 없으면 NULL로 설정하면 됩니다.*/
  );

  xTaskCreate(
    Task2
    ,  "나머지 동작들"
    ,  128
    ,  NULL
    ,  1
    ,  NULL 
   );

   xTaskCreate(
    Task3
    ,  "블루투스"
    ,  128
    ,  NULL
    ,  1
    ,  NULL ); 

}

void loop()
{
  /* 비우도록 합니다. FreeRTOS를 사용할 때에는 사용하지 않습니다. */
}

void Task1(void *pvParameters)
{
  (void) pvParameters;

  Serial.begin(9600);
  pinMode(button_pin1, INPUT);
  pinMode(button_pin2, INPUT);
  pinMode(button_pin3, INPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(digit_select_pin[i], OUTPUT);
  }
  for (int i = 0; i < 8; i++) {
    pinMode(segment_pin[i], OUTPUT);
  }
  time_previous1 = millis();
  int prePowerState = 0;
  lcd.begin(16, 2); // LCD 초기화

  //loop() 시작
  for (;;)
  {
    if (powerState == 1) {
      //if (prePowerState == 0) {
        lcd.display();   //화면 켜기
        //prePowerState == 1;
      //}
      //밝기 권장 안내lcd출력
      if (environment < brightLevel) {
        lcd.clear();
        lcd.setCursor(0, 0); 
        lcd.print("Do it down");
        lcd.setCursor(0, 1);
        lcd.print("Brightness!");
        //lcd에 "밝기낮춤 추천"
      }
      else if (environment > brightLevel) {
        lcd.clear();
        lcd.setCursor(0, 0); 
        lcd.print("Do it up");
        lcd.setCursor(0, 1);
        lcd.print("Brightness!");
        //lcd에 "밝기높힘 추천"
      }
      else if (environment == brightLevel){  //같다면
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Good");
        lcd.setCursor(0, 1);
        lcd.print("Brightness!");
        //lcd에 "적절한 밝기"
      }
      
      state_current = digitalRead(button_pin1);

      if (state_previous == false && state_current == true)
      {
        state_button = !state_button;
      }
      state_previous = state_current;
      state_button_prev = state_button;

      state_current3 = digitalRead(button_pin3);
      if (state_previous3 == false && state_current3 == true)
      {
        state_button3 = !state_button3;
      }
      state_previous3 = state_current3;

      //타이머 실행
      if (state_button == true) {

        if (state_button_prev == false && state_button == true) { //알람->타이머 값 초기화
          hours = 0;
          minutes = 0;
          seconds = 0;
        }

        state_current2 = digitalRead(button_pin2); //타이머 reset
        if (state_previous2 == false && state_current2 == true)
        {
          hours = 0;
          minutes = 0;
          seconds = 0;
        }
        state_previous2 = state_current2;

        //타이머 실행/정지
        if (state_button3 == true) {
          time_current1 = millis();
          if (time_current1 - time_previous1 >= 1000) { // 1초 경과
            time_previous1 = time_current1;
            seconds++; // 초 증가
            if (seconds == 60) { // 60초가 되면 분 증가
              seconds = 0;
              minutes++;
            }
            if (minutes == 60) {
              minutes = 0;
              hours++;// 60분이 되면 0으로 되돌림
            }
          }
        }
        if (hours == 0)
          show_4_digit(minutes * 100 + seconds);// 시간 표시를 위해 4자리 숫자로 만듦
        else
          show_4_digit(hours * 100 + minutes);

        state_button_prev = state_button;
      }



      //알람 실행
      if (state_button == false) {

        if (state_button_prev == true && state_button == false) { //타이머->알람 초기화
          state_button2 = 1;
        }

        if (button(button_pin2)) {
          state_button2++;
          if (state_button2 > 3)
            state_button2 = 1;
        }

        if (state_button2 == 3) { //알람 시작
          time_current1 = millis();
          if (time_current1 - time_previous1 >= 1000) { // 1초 경과
            time_previous1 = time_current1;
            seconds--; // 초 감소
            prev_seconds = seconds;
            if (seconds == 0) {
              if (minutes != 0) {
                seconds = 59;
                minutes--;
              }
            }
            if (seconds < 0) {
              seconds = 0;
            }
          }
          if (seconds == 0 && minutes == 0) { //알람 울림
            int noteLength = 1000 / 4;
            tone(speakerPin, NOTE_E4, noteLength);
            vTaskDelay( 50 / portTICK_PERIOD_MS );
          }
        }
        else if (state_button2 == 1) { //알람 끄기
          noTone(speakerPin);
          seconds = 0;
          hours = 0;
          minutes = 0;
          prev_seconds = 0;
        }
        else if (state_button2 == 2) { //분 설정
          minutes = get_number_from_bluetooth();
          Serial.print("minutes:");
          Serial.println(minutes);
          seconds = get_number_from_bluetooth();
          Serial.print("seconds:");
          Serial.println(seconds);
          state_button2 = 3;
        }
        show_4_digit(minutes * 100 + seconds);
        state_button_prev = state_button;
      }
      prePowerState = 1;  
    }
    if(powerState == 0) {
      if (prePowerState == 1) {
        lcd.noDisplay();
        hours=0;
        minutes=0;
        seconds=0;
        for(int i=0; i<4; i++){
          digitalWrite(digit_select_pin[i], LOW);
        }
        for(int i=0; i<8; i++){
          digitalWrite(segment_pin[i], LOW);
        }
      }
      prePowerState == 0; 
    }
    vTaskDelay(1);
  }
}

void Task2(void *pvParameters)
{
  (void) pvParameters;

  pinMode(powerPin, INPUT);
  pinMode(pirPin, INPUT);
  pinMode(modeButtonPin, INPUT);
  pinMode(echoPin, INPUT);
  pinMode(A2, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
  time_previous = millis();

  for (;;)
  {
    //전원버튼 입력확인, 입력안될시 뒤 동작들 모두 안함.
    if(button(powerPin)) {
      powerCount++;
      timeCount = 0;
    }
    powerState = powerCount % 2;
    //전원off일때 동작들
    if (powerState == 0) {
      //led off
      ledOff();
      //Serial.println("Power off");
    }
    //전원on일때 동작들
    if (powerState == 1) {
      //Serial.println("Power on");
      //led on
      ledOn();
      
      //주변환경의 단계 받아오는 함수
      environment = environmentState(trigPin, echoPin, illuminance);
      
      //버튼 누름 확인후 모드 변경
      if(button(modeButtonPin)) {
        modeButtonCount++;
        timeCount = 0;
        //Serial.println("Mode change");
      }
      selectMode = modeButtonCount % 2;

      //PIR센서의 감지 확인
      time_current2 = millis();
      if (time_current2 - time_previous2 >= 1000) {
        time_previous2 = time_current2;
        pirCurState = digitalRead(pirPin);
        if (pirCurState == pirPreState ) {
          timeCount++;
        }
        else {
          timeCount = 1;
        }
        pirPreState = pirCurState;
        Serial.println(timeCount);
      }
      
      time_current = millis();
      if (time_current - time_previous >= 1000) {
        time_previous = time_current;
        //절전모드(초기모드)
        if (selectMode == 0) {
          if (timeCount >= 8) {  //n초뒤에 절전
            powerSaving();
          }
        }
        //졸음방지모드(버튼 입력시)
        if (selectMode == 1) {
          if (timeCount > 8) {  //n초뒤에 절전
            dontSleep();
          }
        }
      }
    }
    vTaskDelay(1);
  }
}


void Task3(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    char read_data = BTserial.peek();

    // BT –> Data –> Serial
    if (BTserial.available()) {
      Serial.write(read_data);
    }

    if (read_data == 'o') {
      powerCount++;
      BTserial.read();
    }
    if (powerState == 1) {
      if(read_data=='u'){
        brightLevel++;
        BTserial.read();
        if(brightLevel>=5){
          brightLevel=5;
        }
      }
      if(read_data=='d'){
        brightLevel--; 
        BTserial.read();
        if(brightLevel<=1){
          brightLevel=1;
        }
      }
      if(brightLevel==1){
        BRIGHTNESS=51;
      }
      if(brightLevel==2){
        BRIGHTNESS=102;
      }
      if(brightLevel==3){
        BRIGHTNESS=153;
      }
      if(brightLevel==4){
        BRIGHTNESS=204;
      }
      if(brightLevel==5){
        BRIGHTNESS=255;
      }
      ledOn();
    }

    if (powerState == 0) {
      ledOff();
    }

    
    vTaskDelay(1);
  }
  
}
