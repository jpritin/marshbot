//GND - GND
//VCC - VCC
//SDA - Pin D20 - SDA
//SCL - Pin D21 - SCL

//RGB_RED   - Pin 2
//RGB_GREEN - Pin 3
//RGB_BLUE  - Pin 4

 
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define RGB_RED 2
#define RGB_GREEN 3
#define RGB_BLUE 4

#define UMBRAL_VERDE 0.01
#define UMBRAL_AZUL 0.02
#define UMBRAL_AMARILLO 0.03
#define UMBRAL_ROJO 0.04
 
const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);
 
int ax, ay, az;
float v, vmax = -32767, vmin = 32768;
 
 
// Factores de conversion
const float accScale = 2.0 * 9.81 / 32768.0;
const float gyroScale = 250.0 / 32768.0;
 
void printTab()
{
    Serial.print(F("\t"));
}
 
// Mostrar medidas en Sistema Internacional
void printRAW()
{
   Serial.print(F("a[x y] v vmax vmin"));
   Serial.print(ax); printTab();
   Serial.print(ay); printTab();

   v = (abs(ax)/32678.0)*(abs(ay)/32768.0);
   if ( v > vmax) { vmax = v; }
   if ( v < vmin) { vmin = v; }
 
   Serial.print(v); printTab();  
   Serial.print(vmax); printTab();
   Serial.print(vmin); printTab(); 

   // Mostramos semáforo segun nivel v
   if ( v > UMBRAL_ROJO ) {
      Serial.println(F("ROJO"));      
      digitalWrite(RGB_GREEN, LOW); 
      digitalWrite(RGB_BLUE, LOW); 
      digitalWrite(RGB_RED, HIGH);  
   } else if (v > UMBRAL_AMARILLO ) {
      Serial.println(F("AMARILLO"));    
      digitalWrite(RGB_GREEN, HIGH); 
      digitalWrite(RGB_BLUE, LOW); 
      digitalWrite(RGB_RED, HIGH); 
   } else if (v > UMBRAL_AZUL ) {
      Serial.println(F("AZUL"));    
      digitalWrite(RGB_GREEN, LOW); 
      digitalWrite(RGB_BLUE, HIGH); 
      digitalWrite(RGB_RED, LOW); 
   } else {
      Serial.println(F("VERDE"));    
      digitalWrite(RGB_GREEN, HIGH); 
      digitalWrite(RGB_BLUE, LOW); 
      digitalWrite(RGB_RED, LOW); 
   }
}
 
void setup()
{
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
    
    pinMode (RGB_RED, OUTPUT);
    pinMode (RGB_GREEN, OUTPUT);
    pinMode (RGB_BLUE, OUTPUT);   
}
 
void loop()
{
    // Leer las aceleraciones y velocidades angulares
    mpu.getAcceleration(&ax, &ay, &az);
 
    printRAW();
 
    delay(100);
}
