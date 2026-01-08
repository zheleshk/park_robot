void setMotorSpeeds(float lin_x, float lin_y, float ang_z) {
  float geom = L + W;
  
  // lin_x - ВПЕРЕД
  // lin_y - ВЛЕВО
  // ang_z - ВРАЩЕНИЕ ВЛЕВО (против часовой)

  float w1 = (1.0 / R) * (lin_x - lin_y - geom * ang_z); // FL (Переднее Левое)
  float w2 = (1.0 / R) * (lin_x + lin_y + geom * ang_z); // FR (Переднее Правое)
  float w3 = (1.0 / R) * (lin_x + lin_y - geom * ang_z); // BL (Заднее Левое)
  float w4 = (1.0 / R) * (lin_x - lin_y + geom * ang_z); // BR (Заднее Правое)

  // Преобразование в PWM (как и было)
  int pwm1 = map_float(w1, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S, -255, 255);
  int pwm2 = map_float(w2, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S, -255, 255);
  int pwm3 = map_float(w3, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S, -255, 255);
  int pwm4 = map_float(w4, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S, -255, 255);

  set_wheel_pwm(1, pwm1);
  set_wheel_pwm(2, pwm2);
  set_wheel_pwm(3, pwm3);
  set_wheel_pwm(4, pwm4);
}

int map_float(float val, float in_min, float in_max, int out_min, int out_max) {
  return (int)constrain((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}

void set_wheel_pwm(int motor_num, int pwm) {
  int deadzone = 20;
  if (abs(pwm) < deadzone) pwm = 0;
  bool fwd = pwm > 0;
  int abs_pwm = abs(pwm);

  switch (motor_num) {
    case 1: digitalWrite(MFL1A, fwd?HIGH:LOW); digitalWrite(MFL1B, fwd?LOW:HIGH); analogWrite(MFL1S, abs_pwm); break;
    case 2: digitalWrite(MFR2A, fwd?HIGH:LOW); digitalWrite(MFR2B, fwd?LOW:HIGH); analogWrite(MFR2S, abs_pwm); break;
    case 3: digitalWrite(MBL3A, fwd?HIGH:LOW); digitalWrite(MBL3B, fwd?LOW:HIGH); analogWrite(MBL3S, abs_pwm); break;
    case 4: digitalWrite(MBR4A, fwd?HIGH:LOW); digitalWrite(MBR4B, fwd?LOW:HIGH); analogWrite(MBR4S, abs_pwm); break;
  }
}

// Функция для теста: крутит все колеса вперед 5 секунд
void runMotorTest() {
  Serial.println("=== TEST START: MOTORS FORWARD ===");
  
  // Включаем все моторы вперед (PWM 100 из 255 — средняя скорость)
  // Мы используем напрямую set_wheel_pwm, чтобы исключить ошибки в формулах
  set_wheel_pwm(1, 100); 
  set_wheel_pwm(2, 100);
  set_wheel_pwm(3, 100);
  set_wheel_pwm(4, 100);

  // Ждем 5 секунд, пока колеса крутятся
  // В это время смотрите в Serial Monitor на значения <o,...>
  unsigned long start = millis();
  while (millis() - start < 5000) {
    // Продолжаем слать одометрию, чтобы вы видели энкодеры
    noInterrupts();
    long e1 = enc1; long e2 = enc2; long e3 = enc3; long e4 = enc4;
    interrupts();
    Serial.print("<o,");
    Serial.print(e1); Serial.print(",");
    Serial.print(e2); Serial.print(",");
    Serial.print(e3); Serial.print(",");
    Serial.print(e4); 
    Serial.println(">");
    delay(100); // Не частим
  }

  // Останавливаем
  set_wheel_pwm(1, 0); 
  set_wheel_pwm(2, 0);
  set_wheel_pwm(3, 0);
  set_wheel_pwm(4, 0);
  Serial.println("=== TEST END ===");
}