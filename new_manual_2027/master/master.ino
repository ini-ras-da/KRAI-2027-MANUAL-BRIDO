#include <ModbusRtu.h>
#include <Arduino.h>
#include <Ps3Controller.h>

#define EN_PIN 4

#define MAX_SPEED       250
#define BASE_SPEED      100
#define SPEED_INCREMENT 7
#define ACCEL_COEFF     0.3f

// Variabel global
int realSpeed = BASE_SPEED;
int lX = 0, lY = 0, rX = 0, rY = 0;

int targetM[4] = {0, 0, 0, 0};
float currentM[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Setup MODBUS
uint16_t au16dataMotorF[2];
uint16_t au16dataMotorB[2];
uint16_t au16dataSlave3[2];
uint16_t au16dataSlave4[2];
Modbus master(0, Serial, EN_PIN);
modbus_t telegram;

// SLAVE ADDRESS
#define MOTOR_F    1
#define MOTOR_B    2
#define POSEIDON_3 3
#define LIFTER_4   4

// === STATE MACHINE COMBO ===
enum ComboState {
  COMBO_IDLE,
  TRIGGER_POSEIDON,
  WAIT_POSEIDON,
  TRIGGER_LIFTER,
  WAIT_LIFTER
};
ComboState comboState = COMBO_IDLE;
unsigned long lastCheck = 0;

// Variabel manual
bool lastGrip = false;
bool srvState = false; // false = buka, true = tutup

// === FUNCTION DEFINITIONS ===
inline void readInputs();
inline void readInputsR1();
inline void updateMotorSpeeds();
inline void updateMotorSpeedsR1();
void sendToSlave(uint8_t id, int val1, int val2, uint16_t holdingRegister[2]);
void readFromSlave(uint8_t id, uint16_t holdingRegister[2]);
void handleCombo();
void handleManual();

void onConnect() {}
void onDisconnect() {}

void setup() {
  Serial.begin(115200);

  master.start();
  master.setTimeOut(20);

  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin("b4:bf:e9:09:ff:06");

}

void loop() {
  if (Ps3.isConnected()) {
    readInputsR1();
    updateMotorSpeedsR1();

    if (Ps3.data.button.cross && comboState != COMBO_IDLE) {
      // Kirim sinyal 99 buat emergency stop / abort ke kedua slave
      sendToSlave(POSEIDON_3, 99, 0, au16dataSlave3);
      sendToSlave(LIFTER_4, 99, 0, au16dataSlave4);
      comboState = COMBO_IDLE; // Langsung kembali ke manual
    }

    if (Ps3.data.button.square && comboState == COMBO_IDLE) {
      comboState = TRIGGER_POSEIDON;
    }

    if (comboState != COMBO_IDLE) {
      handleCombo();
    } else {
      handleManual(); // Jika auto tidak aktif, kontrol manual siap digunakan
    }

    sendToSlave(MOTOR_F, (int)currentM[0], (int)currentM[1], au16dataMotorF);
    sendToSlave(MOTOR_B, (int)currentM[2], (int)currentM[3], au16dataMotorB);
  }
}

// Alur kontrol manual jika tombol auto tidak ditekan
void handleManual() {
  int cmdPoseidon = 0;
  int cmdLifter = 0;

  // --- MANUAL POSEIDON (SLAVE 3) ---
  if (Ps3.data.button.up) {
    cmdPoseidon = 2; // Manual Maju
  } else if (Ps3.data.button.down) {
    cmdPoseidon = 3; // Manual Mundur
  }

  // Toggle Servo Grip (Segitiga)
  if (Ps3.data.button.triangle) {
    if (!lastGrip) {
      srvState = !srvState;
      lastGrip = true;
    }
    cmdPoseidon = srvState ? 5 : 4; // 4 = Buka, 5 = Tutup
  } else {
    lastGrip = false;
  }

  // --- MANUAL LIFTER (SLAVE 4) ---
  if (Ps3.data.button.up) {
    cmdLifter = 2; // Manual Naik
  } else if (Ps3.data.button.down) {
    cmdLifter = 3; // Manual Turun
  }

  // Kirim perintah manual
  sendToSlave(POSEIDON_3, cmdPoseidon, 0, au16dataSlave3);
  sendToSlave(LIFTER_4, cmdLifter, 0, au16dataSlave4);
}

void handleCombo() {
  switch (comboState) {
    case COMBO_IDLE:
      break;

    case TRIGGER_POSEIDON:
      sendToSlave(POSEIDON_3, 1, 0, au16dataSlave3);
      delay(20);
      comboState = WAIT_POSEIDON;
      break;

    case WAIT_POSEIDON:
      if (millis() - lastCheck > 75) {
        lastCheck = millis();
        readFromSlave(POSEIDON_3, au16dataSlave3);
        if (au16dataSlave3[1] == 0 && au16dataSlave3[0] == 0) {
          comboState = TRIGGER_LIFTER;
        }
      }
      break;

    case TRIGGER_LIFTER:
      sendToSlave(LIFTER_4, 1, 0, au16dataSlave4);
      delay(20);
      comboState = WAIT_LIFTER;
      break;

    case WAIT_LIFTER:
      if (millis() - lastCheck > 75) {
        lastCheck = millis();
        readFromSlave(LIFTER_4, au16dataSlave4);
        if (au16dataSlave4[1] == 0 && au16dataSlave4[0] == 0) {
          comboState = COMBO_IDLE;
        }
      }
      break;
  }
}

// === FUNGSI MOTOR & STICK ===
inline void readInputs() {
  lX = map(Ps3.data.analog.stick.lx, -128, 127, -realSpeed, realSpeed);
  lY = map(Ps3.data.analog.stick.ly, -128, 127, realSpeed, -realSpeed);
  rX = -map(Ps3.data.analog.stick.rx, -128, 127, -realSpeed, realSpeed);
  rY = map(Ps3.data.analog.stick.ry, -128, 127, realSpeed, -realSpeed);

  if (Ps3.data.button.l1) {
    realSpeed = min(realSpeed + SPEED_INCREMENT, MAX_SPEED);
  } else if (Ps3.data.button.l2) {
    realSpeed = max(realSpeed - 60, BASE_SPEED);
  }
}

inline void readInputsR1() {
  lX = map(Ps3.data.analog.stick.lx, -128, 127, -realSpeed, realSpeed);
  lY = map(Ps3.data.analog.stick.ly, -128, 127, realSpeed, -realSpeed);
  rX = -map(Ps3.data.analog.stick.rx, -128, 127, -realSpeed, realSpeed);
  rY = map(Ps3.data.analog.stick.ry, -128, 127, realSpeed, -realSpeed);

  if (Ps3.data.button.l1) {
    realSpeed = min(realSpeed + SPEED_INCREMENT, MAX_SPEED);
  } else if (Ps3.data.button.l2) {
    realSpeed = max(realSpeed - 60, BASE_SPEED);
  }
}

inline void updateMotorSpeeds() {
  float vel = min(sqrt((float)(lX * lX + lY * lY)), (float)realSpeed);
  float arah = atan2(lY, lX);
  float putar = rX * 0.4f;

  targetM[0] = vel * sin(arah - 0.785f) + putar;
  targetM[1] = vel * sin(arah + 0.785f) - putar;
  targetM[2] = -vel * sin(arah - 2.356f) + putar;
  targetM[3] = -vel * sin(arah + 2.356f) - putar;

  for (int i = 0; i < 4; i++) {
    currentM[i] += (targetM[i] - currentM[i]) * ACCEL_COEFF;
    currentM[i] = constrain(currentM[i], -realSpeed, realSpeed);
  }
}

inline void updateMotorSpeedsR1() {
  float vel = min(sqrt((float)(lX * lX + lY * lY)), (float)realSpeed);
  float arah = atan2(lY, lX);
  float putar = rX * 0.4f;

  targetM[0] = vel * sin(arah - 0.785f) + putar;
  targetM[1] = vel * sin(arah + 0.785f) - putar;
  targetM[2] = -vel * sin(arah - 2.356f) + putar;
  targetM[3] = -vel * sin(arah + 2.356f) - putar;

  for (int i = 0; i < 4; i++) {
    currentM[i] += (targetM[i] - currentM[i]) * ACCEL_COEFF;
    currentM[i] = constrain(currentM[i], -realSpeed, realSpeed);
  }
}

void sendToSlave(uint8_t id, int val1, int val2, uint16_t holdingRegister[2]) {
  holdingRegister[0] = (uint16_t)val1;
  holdingRegister[1] = (uint16_t)val2;

  telegram.u8id = id;          
  telegram.u8fct = 16;        
  telegram.u16RegAdd = 0;      
  telegram.u16CoilsNo = 2;    
  telegram.au16reg = holdingRegister;

  master.query(telegram);
  while (master.getState() != COM_IDLE) {
    master.poll();
  }
}

void readFromSlave(uint8_t id, uint16_t holdingRegister[2]) {
  telegram.u8id = id;
  telegram.u8fct = 3;
  telegram.u16RegAdd = 0;
  telegram.u16CoilsNo = 2;
  telegram.au16reg = holdingRegister;

  master.query(telegram);
  while (master.getState() != COM_IDLE) {
    master.poll();
  }
}