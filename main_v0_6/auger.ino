void setAugers(int s1, int s2) {
  if (s1 > 0) { digitalWrite(AUGER1_IN1, HIGH); digitalWrite(AUGER1_IN2, LOW); }
  else if (s1 < 0) { digitalWrite(AUGER1_IN1, LOW); digitalWrite(AUGER1_IN2, HIGH); }
  else { digitalWrite(AUGER1_IN1, LOW); digitalWrite(AUGER1_IN2, LOW); }

  if (s2 > 0) { digitalWrite(AUGER2_IN3, HIGH); digitalWrite(AUGER2_IN4, LOW); }
  else if (s2 < 0) { digitalWrite(AUGER2_IN3, LOW); digitalWrite(AUGER2_IN4, HIGH); }
  else { digitalWrite(AUGER2_IN3, LOW); digitalWrite(AUGER2_IN4, LOW); }
}

void stopAugers() {
  digitalWrite(AUGER1_IN1, LOW); digitalWrite(AUGER1_IN2, LOW);
  digitalWrite(AUGER2_IN3, LOW); digitalWrite(AUGER2_IN4, LOW);
}