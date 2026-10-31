#include <Servo.h>
#include <ModbusRtu.h>

#define TXEN 4

Modbus slave(4, Serial, TXEN);
uint16_t au16data[2] = {0, 0};

const int pinMotor1 = 2;
const int pinMotor2 = 3;

const int pwmMotor1 = 5;
const int pwmMotor2 = 6;

volatile long countEnc = 0;

const int max = 15;   // batas bawah toleransi agar tidak mentok frame
const int min = 490;  // batas atas target naik

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  slave.start();

  pinMode(pinMotor1, INPUT_PULLUP);
  pinMode(pinMotor2, INPUT_PULLUP);
  pinMode(pwmMotor1, OUTPUT);
  pinMode(pwmMotor2, OUTPUT);

  analogWrite(pwmMotor1, 0);
  analogWrite(pwmMotor2, 0);

  attachInterrupt(digitalPinToInterrupt(pinMotor1), readEnc, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  slave.poll(au16data, 2);
  interrupts();

  if (au16data[0] == 1) {
    au16data[1] = 1; // lapor master: lagi gerak
    jalan();
    au16data[0] = 0; // reset perintah
    au16data[1] = 0; // lapor master: selesai (bisa diulang)
  }
}

void jalan() {
  motor_up();
  tahanAtas(3000); // ganti delay biasa biar beban poseidon ditahan aktif
  motor_dwn();

  // reset nol pas di frame bawah biar drift gak numpuk
  noInterrupts();
  countEnc = 0;
  interrupts();
}

void motor_up() {
  while (countEnc < min) {
    slave.poll(au16data, 2);
    analogWrite(pwmMotor1, 0);
    analogWrite(pwmMotor2, 110);
  }
  motorStop();
  delay(100);
}

void tahanAtas(unsigned long jeda) {
  unsigned long tAwal = millis();
  while (millis() - tAwal < jeda) {
    slave.poll(au16data, 2);

    // jika poseidon mulai turun/melorot dari target, tahan naik pelan
    if (countEnc < min - 5) {
      analogWrite(pwmMotor1, 0);
      analogWrite(pwmMotor2, 70);
    } else {
      motorStop();
    }
  }
  motorStop();
}

void motor_dwn() {
  unsigned long tAwal = millis();
  unsigned long cekStall = millis();
  long encLalu = countEnc;

  while (countEnc > max) {
    slave.poll(au16data, 2);

    // deselerasi: pelan saat mendekati frame bawah
    if (countEnc > 130) {
      analogWrite(pwmMotor1, 45);
      analogWrite(pwmMotor2, 0);
    } else {
      analogWrite(pwmMotor1, 20); // soft landing
      analogWrite(pwmMotor2, 0);
    }

    // pengaman: jika 350ms posisi gak gerak (mentok frame), paksa keluar
    if (millis() - cekStall >= 350) {
      if (abs(countEnc - encLalu) < 2) {
        break;
      }
      encLalu = countEnc;
      cekStall = millis();
    }

    // batas waktu turun maksimal 4 detik
    if (millis() - tAwal > 4000) {
      break;
    }
  }
  motorStop();
  delay(100);
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