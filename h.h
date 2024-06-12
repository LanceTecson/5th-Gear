#ifndef h.h
#define h.h

#define BLUEFRUIT_SPI_CS 8
#define BLUEFRUIT_SPI_IRQ 7
#define BLUEFRUIT_SPI_RST 2
#define BLUEFRUIT_SPI_SCK 13
#define BLUEFRUIT_SPI_MISO 12
#define BLUEFRUIT_SPI_MOSI 11

#define BALANCE_EQUIL 1.8  //8


const int DIR_A = 10;
const int DIR_B = 4;
const int PWM = 5;  // Motor B on motor driver

const int DIR_A2 = 0;
const int DIR_B2 = 1;
const int PWM2 = 6;  // MOTOR A on motor driver

const int servoPWM = 9;

const double ANGLE_FIXRATE = 0.1;

const int ERRORMIN = 0;  //0.5
const int ERRORMAX = 5;
const int MOTORMIN = 0;
const int MOTORMAX = 255;
#endif