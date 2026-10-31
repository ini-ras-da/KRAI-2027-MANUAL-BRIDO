#include <Servo.h>
#include <ModbusRtu.h>

#define EN 4

Modbus slave(3, Serial, EN);
uint16_t au16data[2] = {0, 0};

Servo srv1;
Servo srv2;

const int pinMotor1 = 2;
const int pinMotor2 = 3;

const int pwmMotor1 = 5;
const int pwmMotor2 = 6;  

volatile long countEnc = 0;

const int max = -10;   // batas mundur ke awal
const int min = -900;  // batas maju penuh

void setup() {
  Serial.begin(115200);
  slave.start();

  srv1.attach(10);
  srv2.attach(9);

  srv_buka();

  pinMode(pinMotor1, INPUT_PULLUP);
  pinMode(pinMotor2, INPUT_PULLUP);
  pinMode(pwmMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);

  analogWrite(pwmMotor1, 0);
  analogWrite(pwmMotor2, 0);

  attachInterrupt(digitalPinToInterrupt(pinMotor1), readEnc, RISING);
}

void loop() {
  slave.poll(au16data, 2);
  interrupts();

  // AUTO COMBO
  if (au16data[0] == 1) {
    au16data[1] = 1; // lapor master: lagi gerak
    jalanPoseidon();
    au16data[0] = 0;
    au16data[1] = 0; // lapor master: selesai
  }
  // MANUAL MAJU
  else if (au16data[0] == 2) {
    if (countEnc > min) {
      analogWrite(pwmMotor1, 0);
      analogWrite(pwmMotor2, 80);
    } else {
      motorStop();
    }
  }
  // MANUAL MUNDUR
  else if (au16data[0] == 3) {
    if (countEnc < max) {
      analogWrite(pwmMotor1, 80);
      analogWrite(pwmMotor2, 0);
    } else {
      motorStop();
    }
  }
  // MANUAL SERVO BUKA
  else if (au16data[0] == 4) {
    srv_buka();
  }
  // MANUAL SERVO TUTUP
  else if (au16data[0] == 5) {
    srv_tutup();
  }
  // EMERGENCY ABORT / STOP
  else if (au16data[0] == 99 || au16data[0] == 0) {
    motorStop();
    au16data[1] = 0;
  }
}

void jalanPoseidon() {
  srv_buka();
  motor_f();
  if (au16data[0] == 99) return; // langsung keluar jika di-override

  srv_tutup();
  motor_b();

  // reset titik 0
  noInterrupts();
  countEnc = 0;
  interrupts();
}

void motor_f() {
  unsigned long tAwal = millis();
  unsigned long cekStall = millis();
  long encLalu = countEnc;

  while (countEnc > min) {
    slave.poll(au16data, 2);
    if (au16data[0] == 99) { motorStop(); break; } // cek sinyal abort dari stik

    analogWrite(pwmMotor1, 0);
    analogWrite(pwmMotor2, 90);

    if (millis() - cekStall >= 350) {
      if (abs(countEnc - encLalu) < 2) break;
      encLalu = countEnc;
      cekStall = millis();
    }

    if (millis() - tAwal > 3500) break;
  }
  motorStop();
  delayWithPoll(150);
}

void motor_b() {
  unsigned long tAwal = millis();
  unsigned long cekStall = millis();
  long encLalu = countEnc;

  while (countEnc < max) {
    slave.poll(au16data, 2);
    if (au16data[0] == 99) { motorStop(); break; } // cek sinyal abort dari stik

    analogWrite(pwmMotor1, 90);
    analogWrite(pwmMotor2, 0);

    if (millis() - cekStall >= 350) {
      if (abs(countEnc - encLalu) < 2) break;
      encLalu = countEnc;
      cekStall = millis();
    }

    if (millis() - tAwal > 3500) break;
  }
  motorStop();
  delayWithPoll(150);
}

void srv_tutup() {
  srv1.write(100);
  srv2.write(60);
  delayWithPoll(300);
}

void srv_buka() {
  srv1.write(180);
  srv2.write(0);
  delayWithPoll(300);
}

void delayWithPoll(unsigned long jeda) {
  unsigned long tAwal = millis();
  while (millis() - tAwal < jeda) {
    slave.poll(au16data, 2);
    if (au16data[0] == 99) break;
  }
}

void motorStop() {
  analogWrite(pwmMotor1, 0);
  analogWrite(pwmMotor2, 0);
  delay(1);
}

void readEnc() {
  if (digitalRead(pinMotor2)) {
    countEnc++;
  } else {
    countEnc--;
  }
}