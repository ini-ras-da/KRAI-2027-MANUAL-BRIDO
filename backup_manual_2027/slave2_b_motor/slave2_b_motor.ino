#include <ModbusRtu.h>

// Pin
#define rpw1 5
#define lpw1 3
#define rpw2 6
#define lpw2 11
#define en 4 

// Data Modbus 
uint16_t au16data[2] = {0, 0};

// Inisiasi Modbus Slave 2
Modbus slave(2, Serial, en); 

// Motor
void updateMotor(){
  int m = (int16_t)au16data[0];
  int k = (int16_t)au16data[1];

  if (m > 0){
    analogWrite(rpw1, 0);
    analogWrite(lpw1, m);
  }
  else {
    analogWrite(rpw1, -m);
    analogWrite(lpw1, 0);
  }

  if (k > 0){
    analogWrite(rpw2, k);
    analogWrite(lpw2, 0);
  }
  else {
    analogWrite(rpw2, 0);
    analogWrite(lpw2, -k);
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(rpw1, OUTPUT);
  pinMode(rpw2, OUTPUT);
  pinMode(lpw1, OUTPUT);
  pinMode(lpw2, OUTPUT);
  pinMode(en, OUTPUT);
  
  slave.start();
}

void loop() {
  // Pengambilan Data Master
  slave.poll(au16data, 2);
  
  updateMotor();
}