/*** Arduino- IR sensor***/
/*
      8x8 LED Matrix MAX7219 Scrolling Text 

   Based on the following library:
   GitHub | riyas-org/max7219  https://github.com/riyas-org/max7219
*/

/*
This file is merely an aggregation of Servo, Display, IR Sensor to operate a demo. It shows my learning skills 
*/

#include <Servo.h>
#include <MaxMatrix.h>
#include <avr/pgmspace.h>

PROGMEM const unsigned char CH[] = {
  3, 8, B00000000, B00000000, B00000000, B00000000, B00000000, // space
  1, 8, B01011111, B00000000, B00000000, B00000000, B00000000, // !
  3, 8, B00000011, B00000000, B00000011, B00000000, B00000000, // "
  5, 8, B00010100, B00111110, B00010100, B00111110, B00010100, // #
  4, 8, B00100100, B01101010, B00101011, B00010010, B00000000, // $
  5, 8, B01100011, B00010011, B00001000, B01100100, B01100011, // %
  5, 8, B00110110, B01001001, B01010110, B00100000, B01010000, // &
  1, 8, B00000011, B00000000, B00000000, B00000000, B00000000, // '
  3, 8, B00011100, B00100010, B01000001, B00000000, B00000000, // (
  3, 8, B01000001, B00100010, B00011100, B00000000, B00000000, // )
  5, 8, B00101000, B00011000, B00001110, B00011000, B00101000, // *
  5, 8, B00001000, B00001000, B00111110, B00001000, B00001000, // +
  2, 8, B10110000, B01110000, B00000000, B00000000, B00000000, // ,
  4, 8, B00001000, B00001000, B00001000, B00001000, B00000000, // -
  2, 8, B01100000, B01100000, B00000000, B00000000, B00000000, // .
  4, 8, B01100000, B00011000, B00000110, B00000001, B00000000, // /
  4, 8, B00111110, B01000001, B01000001, B00111110, B00000000, // 0
  3, 8, B01000010, B01111111, B01000000, B00000000, B00000000, // 1
  4, 8, B01100010, B01010001, B01001001, B01000110, B00000000, // 2
  4, 8, B00100010, B01000001, B01001001, B00110110, B00000000, // 3
  4, 8, B00011000, B00010100, B00010010, B01111111, B00000000, // 4
  4, 8, B00100111, B01000101, B01000101, B00111001, B00000000, // 5
  4, 8, B00111110, B01001001, B01001001, B00110000, B00000000, // 6
  4, 8, B01100001, B00010001, B00001001, B00000111, B00000000, // 7
  4, 8, B00110110, B01001001, B01001001, B00110110, B00000000, // 8
  4, 8, B00000110, B01001001, B01001001, B00111110, B00000000, // 9
  2, 8, B01010000, B00000000, B00000000, B00000000, B00000000, // :
  2, 8, B10000000, B01010000, B00000000, B00000000, B00000000, // ;
  3, 8, B00010000, B00101000, B01000100, B00000000, B00000000, // <
  3, 8, B00010100, B00010100, B00010100, B00000000, B00000000, // =
  3, 8, B01000100, B00101000, B00010000, B00000000, B00000000, // >
  4, 8, B00000010, B01011001, B00001001, B00000110, B00000000, // ?
  5, 8, B00111110, B01001001, B01010101, B01011101, B00001110, // @
  4, 8, B01111110, B00010001, B00010001, B01111110, B00000000, // A
  4, 8, B01111111, B01001001, B01001001, B00110110, B00000000, // B
  4, 8, B00111110, B01000001, B01000001, B00100010, B00000000, // C
  4, 8, B01111111, B01000001, B01000001, B00111110, B00000000, // D
  4, 8, B01111111, B01001001, B01001001, B01000001, B00000000, // E
  4, 8, B01111111, B00001001, B00001001, B00000001, B00000000, // F
  4, 8, B00111110, B01000001, B01001001, B01111010, B00000000, // G
  4, 8, B01111111, B00001000, B00001000, B01111111, B00000000, // H
  3, 8, B01000001, B01111111, B01000001, B00000000, B00000000, // I
  4, 8, B00110000, B01000000, B01000001, B00111111, B00000000, // J
  4, 8, B01111111, B00001000, B00010100, B01100011, B00000000, // K
  4, 8, B01111111, B01000000, B01000000, B01000000, B00000000, // L
  5, 8, B01111111, B00000010, B00001100, B00000010, B01111111, // M
  5, 8, B01111111, B00000100, B00001000, B00010000, B01111111, // N
  4, 8, B00111110, B01000001, B01000001, B00111110, B00000000, // O
  4, 8, B01111111, B00001001, B00001001, B00000110, B00000000, // P
  4, 8, B00111110, B01000001, B01000001, B10111110, B00000000, // Q
  4, 8, B01111111, B00001001, B00001001, B01110110, B00000000, // R
  4, 8, B01000110, B01001001, B01001001, B00110010, B00000000, // S
  5, 8, B00000001, B00000001, B01111111, B00000001, B00000001, // T
  4, 8, B00111111, B01000000, B01000000, B00111111, B00000000, // U
  5, 8, B00001111, B00110000, B01000000, B00110000, B00001111, // V
  5, 8, B00111111, B01000000, B00111000, B01000000, B00111111, // W
  5, 8, B01100011, B00010100, B00001000, B00010100, B01100011, // X
  5, 8, B00000111, B00001000, B01110000, B00001000, B00000111, // Y
  4, 8, B01100001, B01010001, B01001001, B01000111, B00000000, // Z
  2, 8, B01111111, B01000001, B00000000, B00000000, B00000000, // [
  4, 8, B00000001, B00000110, B00011000, B01100000, B00000000, // \ backslash
  2, 8, B01000001, B01111111, B00000000, B00000000, B00000000, // ]
  3, 8, B00000010, B00000001, B00000010, B00000000, B00000000, // hat
  4, 8, B01000000, B01000000, B01000000, B01000000, B00000000, // _
  2, 8, B00000001, B00000010, B00000000, B00000000, B00000000, // `
  4, 8, B00100000, B01010100, B01010100, B01111000, B00000000, // a
  4, 8, B01111111, B01000100, B01000100, B00111000, B00000000, // b
  4, 8, B00111000, B01000100, B01000100, B00101000, B00000000, // c
  4, 8, B00111000, B01000100, B01000100, B01111111, B00000000, // d
  4, 8, B00111000, B01010100, B01010100, B00011000, B00000000, // e
  3, 8, B00000100, B01111110, B00000101, B00000000, B00000000, // f
  4, 8, B10011000, B10100100, B10100100, B01111000, B00000000, // g
  4, 8, B01111111, B00000100, B00000100, B01111000, B00000000, // h
  3, 8, B01000100, B01111101, B01000000, B00000000, B00000000, // i
  4, 8, B01000000, B10000000, B10000100, B01111101, B00000000, // j
  4, 8, B01111111, B00010000, B00101000, B01000100, B00000000, // k
  3, 8, B01000001, B01111111, B01000000, B00000000, B00000000, // l
  5, 8, B01111100, B00000100, B01111100, B00000100, B01111000, // m
  4, 8, B01111100, B00000100, B00000100, B01111000, B00000000, // n
  4, 8, B00111000, B01000100, B01000100, B00111000, B00000000, // o
  4, 8, B11111100, B00100100, B00100100, B00011000, B00000000, // p
  4, 8, B00011000, B00100100, B00100100, B11111100, B00000000, // q
  4, 8, B01111100, B00001000, B00000100, B00000100, B00000000, // r
  4, 8, B01001000, B01010100, B01010100, B00100100, B00000000, // s
  3, 8, B00000100, B00111111, B01000100, B00000000, B00000000, // t
  4, 8, B00111100, B01000000, B01000000, B01111100, B00000000, // u
  5, 8, B00011100, B00100000, B01000000, B00100000, B00011100, // v
  5, 8, B00111100, B01000000, B00111100, B01000000, B00111100, // w
  5, 8, B01000100, B00101000, B00010000, B00101000, B01000100, // x
  4, 8, B10011100, B10100000, B10100000, B01111100, B00000000, // y
  3, 8, B01100100, B01010100, B01001100, B00000000, B00000000, // z
  3, 8, B00001000, B00110110, B01000001, B00000000, B00000000, // {
  1, 8, B01111111, B00000000, B00000000, B00000000, B00000000, // |
  3, 8, B01000001, B00110110, B00001000, B00000000, B00000000, // }
  4, 8, B00001000, B00000100, B00001000, B00000100, B00000000, // ~
};

int DIN = 12;   // DIN GPIO pin of MAX7219 module
int CLK = 13;   // CLK GPIO pin of MAX7219 module
int CS = 3;    // CS GPIO pin of MAX7219 module
int maxInUse = 2;
int NUMPLSENSORS = 4;
char vacancylist[5];

MaxMatrix m(DIN, CS, CLK, maxInUse);

byte buffer[10];

char text[]= " Welcome to Parking Demo "; 

void printCharWithShift(char c, int shift_speed) {
  if (c < 32) return;
  c -= 32;
  memcpy_P(buffer, CH + 7 * c, 7);
  m.writeSprite(32, 0, buffer);
  m.setColumn(32 + buffer[0], 0);

  for (int i = 0; i < buffer[0] + 1; i++)
  {
    delay(shift_speed);
    m.shiftLeft(false, false);
  }
}
// Extract the characters from the text string
void printStringWithShift(char* s, int shift_speed) {
  while (*s != 0) {
    printCharWithShift(*s, shift_speed);
    s++;
  }
}

Servo myservo;
int pos = 0;

int Sensor1Pin = 5;
int Sensor2Pin = 4;
int Park1Sensor = 2;
int Park2Sensor = 51;
int Park3Sensor = 53;
int Park4Sensor = 49;
char vacantslot=0;

void setup() {
  //pinMode(OutputPin, OUTPUT);

  pinMode(Sensor1Pin, INPUT);
  pinMode(Sensor2Pin, INPUT);
  pinMode(Park1Sensor, INPUT);
  pinMode(Park2Sensor, INPUT);
  pinMode(Park3Sensor, INPUT);
  pinMode(Park4Sensor, INPUT); 

  myservo.attach(9);
  myservo.write(90);
  Serial.begin(9600);

  m.init(); // module initialize
  m.setIntensity(0); // dot matix intensity 0-15
    printStringWithShift(text, 100);
  delay(1000);
}

void loop() {
  char c[10];
  int Sensor1Value = digitalRead(Sensor1Pin);
  int Sensor2Value = digitalRead(Sensor2Pin);
  vacantslot = 0;
  Serial.print("Sensor1Pin Value =");
  Serial.println(Sensor1Value);

  Serial.print("Sensor2Pin Value =");
  Serial.println(Sensor2Value);
  
  if (Sensor1Value==LOW){ // LOW MEANS Object Detected
  findvacancy();
  memset(c,0,sizeof(c));
  printStringWithShift(c, 100);

  if((int)vacantslot == 1){
    c[0] = '1';
    c[1] = ' ';
    c[2] = '1';
    c[3] = ' ';
  }
  else if((int)vacantslot == 2){
    c[0] = '2';
    c[1] = ' ';
    c[2] = '2';
    c[3] = ' ';
  }
  else if((int)vacantslot == 3){
    c[0] = '3';
    c[1] = ' ';
    c[2] = '3';
    c[3] = ' ';
  }
  else if((int)vacantslot == 4){
    c[0] = '4';
    c[1] = ' ';
    c[2] = '4';
    c[3] = ' ';
  }
  else if((int)vacantslot == 0){
    c[0] = 'F';
    c[1] = 'U';
    c[2] = 'L';
    c[3] = 'L';
  }

  //digitalWrite(OutputPin, HIGH); // HIGH MEANS No object interference
  //Display a welcome message
  if((int)vacantslot != 0)
      myservo.write(0);

/*  for (pos = 90; pos >= 0; pos -=1) {
      myservo.write(pos);
      delay(15);
  }*/
 Serial.println("vacantslot");
 Serial.println((int)vacantslot);

  Serial.println(c);
  Serial.println("Waiting at Gate");
  Serial.println((int)vacantslot);
  while(digitalRead(Sensor2Pin)==HIGH){
  
  printStringWithShift(c, 100);
  }
  Serial.println("Out of the gate");
  Sensor2Value = digitalRead(Sensor2Pin);

  if (Sensor2Value==LOW){ // LOW MEANS Object Detected
/*  for (pos = 0; pos <= 90; pos += 1) {
    myservo.write(pos);
    delay(15);
  }*/
        myservo.write(90);
  delay(100);
   }
  }  
} 

void findvacancy()
{
  if(digitalRead(Park1Sensor)){
     Serial.println("Park1");
    vacantslot = 1;
    return;}
  else if(digitalRead(Park2Sensor)){
    Serial.println("Park2");
    vacantslot = 2;
    return;}
  else if(digitalRead(Park3Sensor)){
    Serial.println("Park3");
    vacantslot = 3;
    return;}
  else if(digitalRead(Park4Sensor)){
    Serial.println("Park4");
    vacantslot = 4;
    return;}
  else
    vacantslot = 0;
    return;

}