// Interrupt info for wheel encoders
const int RPM_PIN_L_F = 0;
const int RPM_PIN_R_F = 9;
const int RPM_PIN_L_R = 10;
const int RPM_PIN_R_R = 11;

//RPM Counters
volatile int count_L_F = 0;
volatile int count_R_F = 0;
volatile int count_L_R = 0;
volatile int count_R_R = 0;

//Semaphore
portMUX_TYPE mmux = portMUX_INITIALIZER_UNLOCKED;

//Check the revolutions for each wheel
void IRAM_ATTR rpm() {
  if (digitalRead(RPM_PIN_L_F) == 0) {
    taskENTER_CRITICAL(&mmux);
    count_L_F++;
    doc["olf"] = count_L_F;
    taskEXIT_CRITICAL(&mmux);
  }
  if (digitalRead(RPM_PIN_R_F) == 0) {
    taskENTER_CRITICAL(&mmux);
    count_R_F++;
    doc["orf"] = count_R_F;
    taskEXIT_CRITICAL(&mmux);
  }
  if (digitalRead(RPM_PIN_L_R) == 0) {
    taskENTER_CRITICAL(&mmux);
    count_L_R++;
    doc["olr"] = count_L_R;
    taskEXIT_CRITICAL(&mmux);
  }
  if (digitalRead(RPM_PIN_R_R) == 0) {
    taskENTER_CRITICAL(&mmux);
    count_R_R++;
    doc["orr"] = count_R_R;
    taskEXIT_CRITICAL(&mmux);
  }
}

void attachInterrupts() {
  pinMode(RPM_PIN_L_R, INPUT_PULLUP);
  pinMode(RPM_PIN_R_R, INPUT_PULLUP);
  pinMode(RPM_PIN_L_F, INPUT_PULLUP);
  pinMode(RPM_PIN_R_F, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RPM_PIN_L_F), rpm, LOW);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN_R_F), rpm, LOW);

  attachInterrupt(digitalPinToInterrupt(RPM_PIN_L_R), rpm, LOW);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN_R_R), rpm, LOW);
}
