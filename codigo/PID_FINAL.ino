#include <Wire.h>
//Se defines los pines de la esp a los que van conectados los cables de señal de la esc
const int pinMotorLeft = 4;  
const int pinMotorRight = 15; 
//fRECuencia de la señal PWM
const int freq = 50;
//NÚMERO DE BITS DEL PWM
const int resolution = 16; 

//Declaramos variables de 16 bits

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

//Declara un arreglo 

float Total_angle[2]; 

// Variables para medir el tiempo, con unsigned long porque solo toman valores positivos
unsigned long timeCurr, timePrev; 
float elapsedTime;
//Se definen variables FLOAT Con respecto al PID
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0, pid_i=0, pid_d=0;

//Como una ESC es diferente, necesita recibir una señal PWM mayor para iniciar como el otro motor 
int offset_left = 150;  

// El motor cuando se acerca al setpoint disminuye throttle, cuando esta abajo aumenta y cuando esta arriba disminuye
int direction = 1; 

//Valor inicial de thrttle
double throttle = 1150; 

// CONSTANTES PID INICIALES
double kp = 3.0; 
double ki = 0.02;
double kd = 1.0; 
//SETPOINT
float desired_angle = 0;

// Convierte los microsegundos al valor digital que entiende la ESP32
void writeESC(int pin, int microseconds) {
  int duty = (microseconds * 65535) / 20000; 
  ledcWrite(pin, duty); 
}

void setup() {
  //Se definen los pines de la ESP para la comunicación I2C con el sensor
  Wire.begin(21, 22);
  //Se comunica con la MPU
  Wire.beginTransmission(0x68);
   Wire.write(0x6B); 
   Wire.write(0); 
   Wire.endTransmission(true);
  //Se inicia el monitor serial
  Serial.begin(115200);
//Configuración asigna los pines a los que enviar la señal PWM para las esc's
  ledcAttach(pinMotorLeft, freq, resolution);
  ledcAttach(pinMotorRight, freq, resolution);
// Envia una señal de mínimo a la ESC por 4 segundos 
  Serial.println("ARMANDO MOTORES (4 seg)...");
  writeESC(pinMotorLeft, 1000);
  writeESC(pinMotorRight, 1000);
  delay(4000); 
  //Al pasar los 4 segundos inicia el PID
  Serial.println("¡PID ACTIVO!");
  timeCurr = millis();
}

void loop() {
  //controla la frencuencia del sistema
  if (millis() - timeCurr < 10) return;
  //se guardan los tiempos del ciclo anterior
  timePrev = timeCurr;
  timeCurr = millis();
  //y se calcula cuanto tardó el ciclo
  elapsedTime = (timeCurr - timePrev) / 1000.0;
  //por seguridad para evitar cosas infinitas
  if(elapsedTime == 0) elapsedTime = 0.001;

//IMU. Lee el sensor MPU6050
//0x68 ES LA DIRECCIÓN I2C DEL MPU
  Wire.beginTransmission(0x68); 
//Es el registro donde empieza la lectura del sesor
  Wire.write(0x3B); 
  //termina la transmisión sin liberar el bus de comunicación para no tener que ejecutar el código de nuevo
  Wire.endTransmission(false);
  //PIDE 6 BYTES DEL MPU LA ACCX,ACCY,ACCZ
  Wire.requestFrom(0x68,6,true);
  Acc_rawX=Wire.read()<<8|Wire.read();
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();
  
// Conversión de los datos del acelerometro a grados 
  float angle_pitch = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*57.296;
//SE PIDEN LOS DATOS ANGULARES DEL GIROSCOPIO
  Wire.beginTransmission(0x68); 
  //0X43 ES el registro inicial del gyro
  Wire.write(0x43); 
  Wire.endTransmission(false);
  //SOLO PIDE 4 BYTES A LA MPU
  Wire.requestFrom(0x68,4,true); 
  Gyr_rawX=Wire.read()<<8|Wire.read();
  Gyr_rawY=Wire.read()<<8|Wire.read();
  
  //FILTRO COMPLEMENTARIO del acelerometro y el giroscopio para mejorar la lectura de la posición 
  float Gyro_rate = Gyr_rawY/131.0; 
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_rate*elapsedTime) + 0.02*angle_pitch;
  
  // PID
  error = Total_angle[1] - desired_angle;
  pid_p = kp*error;
  if(abs(error) < 10) pid_i += ki*error; else pid_i = 0;
  pid_d = kd*((error - previous_error)/elapsedTime);
  PID = pid_p + pid_i + pid_d;
  
  //Limitación del PID 
  if(PID < -400) PID=-400; if(PID > 400) PID=400;

//SE DEFINEN LAS SEÑALES CON SIGNOS OPUESTOS
  pwmLeft = throttle + (PID * direction);
  pwmRight = throttle - (PID * direction);

//SE LE SUMA EL OFFSET AL MOTOR DE LA IZQUIERDA 
  pwmLeft = pwmLeft + offset_left;

// Se fija un mínimo y un máximo de PWM 
  if(pwmRight < 1050) pwmRight=1050; if(pwmRight > 1950) pwmRight=1950;
  if(pwmLeft < 1050) pwmLeft=1050;   if(pwmLeft > 1950) pwmLeft=1950;

//Seguridad QUE NO EXCEDA EL ANGULO DE 55
  if(abs(Total_angle[1]) > 55) { pwmLeft=1000; pwmRight=1000; }
//ENVIA EL PWM A LAS ESC'S
  writeESC(pinMotorLeft, pwmLeft);
  writeESC(pinMotorRight, pwmRight);
//GUARDA EL ERROR ANTERIOR
  previous_error = error;
//PRINTS PARA NO SATURAR EL MONITOR SERIAL
  static int printCount = 0;
  printCount++;
  if(printCount > 20) {
    Serial.print("Ang:"); Serial.print(Total_angle[1]);
    Serial.print(" | PID:"); Serial.print(PID);
    Serial.print(" | L(4):"); Serial.print(pwmLeft);
    Serial.print(" | R(15):"); Serial.println(pwmRight);
    printCount = 0;
  }
}