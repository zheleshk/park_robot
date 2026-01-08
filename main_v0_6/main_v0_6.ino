//моторы: F-forward, B-back, L-left, R-right
//A-IN1, B-IN2, S-PWM

// Пины моторов
const int MFL1A = 5, MFL1B = 3, MFL1S = 7;
const int MFR2A = 15, MFR2B = 17, MFR2S = 9;
const int MBL3A = 25, MBL3B = 23, MBL3S = 11;
const int MBR4A = 27, MBR4B = 29, MBR4S = 13;

// Пины энкодеров (ВАШИ НОВЫЕ НАСТРОЙКИ)
const int EFL1_A = 19; const int EFL1_B = 33;
const int EFR2_A = 21; const int EFR2_B = 35;
const int EBL3_A = 2;  const int EBL3_B = 41; // Пин 2
const int EBR4_A = 20; const int EBR4_B = 43; // Пин 20

// Пины шнеков
const int AUGER1_IN1 = 36;
const int AUGER1_IN2 = 38;
const int AUGER2_IN3 = 44;
const int AUGER2_IN4 = 46;

// Параметры робота
const double L = 0.13; 
const double W = 0.135;
const double R = 0.08;
const float MAX_WHEEL_RAD_S = 8.06; 

volatile long enc1=0, enc2=0, enc3=0, enc4=0;
String incoming_data = "";

void setup() {
  Serial.begin(115200);

  // Настройка моторов
  pinMode(MFL1A, OUTPUT); pinMode(MFL1B, OUTPUT); pinMode(MFL1S, OUTPUT);
  pinMode(MFR2A, OUTPUT); pinMode(MFR2B, OUTPUT); pinMode(MFR2S, OUTPUT);
  pinMode(MBL3A, OUTPUT); pinMode(MBL3B, OUTPUT); pinMode(MBL3S, OUTPUT);
  pinMode(MBR4A, OUTPUT); pinMode(MBR4B, OUTPUT); pinMode(MBR4S, OUTPUT);

  // Настройка энкодеров
  pinMode(EFL1_A, INPUT_PULLUP); pinMode(EFL1_B, INPUT_PULLUP);
  pinMode(EFR2_A, INPUT_PULLUP); pinMode(EFR2_B, INPUT_PULLUP);
  pinMode(EBL3_A, INPUT_PULLUP); pinMode(EBL3_B, INPUT_PULLUP);
  pinMode(EBR4_A, INPUT_PULLUP); pinMode(EBR4_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EFL1_A), countEncoder1, FALLING);
  attachInterrupt(digitalPinToInterrupt(EFR2_A), countEncoder2, FALLING);
  attachInterrupt(digitalPinToInterrupt(EBL3_A), countEncoder3, FALLING);
  attachInterrupt(digitalPinToInterrupt(EBR4_A), countEncoder4, FALLING);

  // Настройка шнеков
  pinMode(AUGER1_IN1, OUTPUT); pinMode(AUGER1_IN2, OUTPUT);
  pinMode(AUGER2_IN3, OUTPUT); pinMode(AUGER2_IN4, OUTPUT);
  stopAugers();
  
  // !!! ТЕСТ УБРАН, ЧТОБЫ РАБОТАЛ ROS !!!
}

void loop() {
  // Чтение данных из Serial (от Raspberry Pi)
  while (Serial.available() > 0) {
    char received_char = Serial.read();
    if (received_char == '>') {
      parseCommand(incoming_data);
      incoming_data = "";
    } else if (received_char == '<') {
      incoming_data = "";
    } else {
      incoming_data += received_char;
    }
  }

  // Отправка одометрии (50 Гц)
  static unsigned long last_odom_time = 0;
  if (millis() - last_odom_time > 20) { 
    noInterrupts();
    long e1 = enc1; long e2 = enc2; long e3 = enc3; long e4 = enc4;
    interrupts();

    Serial.print("<o,");
    Serial.print(e1); Serial.print(",");
    Serial.print(e2); Serial.print(",");
    Serial.print(e3); Serial.print(",");
    Serial.print(e4); 
    Serial.println(">");
    last_odom_time = millis();
  }
}

// ВАШИ ИСПРАВЛЕННЫЕ ФУНКЦИИ ЭНКОДЕРОВ
void countEncoder1() { if (digitalRead(EFL1_B) == LOW) enc1--; else enc1++; }
void countEncoder2() { if (digitalRead(EFR2_B) == LOW) enc2++; else enc2--; }
void countEncoder3() { if (digitalRead(EBL3_B) == LOW) enc3--; else enc3++; }
void countEncoder4() { if (digitalRead(EBR4_B) == LOW) enc4++; else enc4--; }

// Парсинг команд
void parseCommand(String data) {
  if (data.startsWith("v,")) {
    data.remove(0, 2);
    int c1 = data.indexOf(',');
    int c2 = data.indexOf(',', c1 + 1);
    if (c1 == -1 || c2 == -1) return;

    float lx = data.substring(0, c1).toFloat();
    float ly = data.substring(c1 + 1, c2).toFloat();
    float wz = data.substring(c2 + 1).toFloat();
    
    setMotorSpeeds(lx, ly, wz);
  }
  else if (data.startsWith("a,")) {
    data.remove(0, 2);
    int c1 = data.indexOf(',');
    if (c1 == -1) return;
    int auger1_state = data.substring(0, c1).toInt();
    int auger2_state = data.substring(c1 + 1).toInt();
    setAugers(auger1_state, auger2_state);
  }
}