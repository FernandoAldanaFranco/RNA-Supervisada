//RNA aprendizaje supervisado barco recoletctor de basura.
//2024
//Fernando Aldana y Jesús Morales
//Universidad Veracruzana

//ESC de motores, MotorIzq es Motor Izquierdo, MotorDer es Motor Derecho
#include <Servo.h> 
Servo MotorL; //Motores izquierdo
Servo MotorR; //Derecho

//Ultrasónico 1
const int Trigger1 = 48;  
const int Echo1 = 49;     
//Ultrasónico 2
const int Trigger2 = 46;  
const int Echo2 = 47;     
//Ultrasónico 3
const int Trigger3 = 44;  
const int Echo3 = 45;     
//Ultrasónico 4
const int Trigger4 = 42;  
const int Echo4 = 43; 
//Lado Derecho
//Ultrasónico 5
const int Trigger5 = 38;  
const int Echo5 = 39;     
//Ultrasónico 6
const int Trigger6 = 36;  
const int Echo6 = 37;     
//Ultrasónico 7
const int Trigger7 = 34;  
const int Echo7 = 35;     
//Ultrasónico 8
const int Trigger8 = 32;  
const int Echo8 = 33;     

//Lado Izquierdo
//Infrarrojo 1
const int sensorPin1 = 3;
//Infrarrojo 2
const int sensorPin2 = 4;
//Infrarrojo 3
const int sensorPin3 = 5;
//Infrarrojo 4
const int sensorPin4 = 6;
//Lado Derecho
//Infrarrojo 5
const int sensorPin5 = 50;
//Infrarrojo 6
const int sensorPin6 = 51;
//Infrarrojo 7
const int sensorPin7 = 53;
//Infrarrojo 8
const int sensorPin8 = 52;


//RNA
const int inputNumber=6; //Numero de neuronas en capa de entrada
const int outputNumber=5; //Numero de neuronas en salida
const int sensorsNumber=8; //sensores, en este caso solo IR
const int weightsNumber=(inputNumber*(sensorsNumber+1))+(outputNumber*(inputNumber+1)); //Cantidad de pesos
const int numberNeurons=inputNumber+outputNumber;
float weights[weightsNumber]; //Guarda los pesos de la RNA
float net[numberNeurons]; //Para cálculo de net
float TF[numberNeurons]; //FT
float output[numberNeurons]; //Salidas
float sensors[sensorsNumber]; //Estado de sensores

void setup() {
  pinMode(Trigger1, OUTPUT);    //pin como salida     
  pinMode(Echo1, INPUT);        //pin como entrada
  pinMode(Trigger2, OUTPUT);    
  pinMode(Echo2, INPUT);        
  pinMode(Trigger3, OUTPUT);    
  pinMode(Echo3, INPUT);        
  pinMode(Trigger4, OUTPUT);    
  pinMode(Echo4, INPUT);        
  pinMode(Trigger5, OUTPUT);    
  pinMode(Echo5, INPUT);        
  pinMode(Trigger6, OUTPUT);    
  pinMode(Echo6, INPUT);
  pinMode(Trigger7, OUTPUT);    
  pinMode(Echo7, INPUT);
  pinMode(Trigger8, OUTPUT);    
  pinMode(Echo8, INPUT);        
       
  digitalWrite(Trigger1, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger2, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger3, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger4, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger5, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger6, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger7, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger8, LOW);  //Inicializamos el pin con 0

  //Infrarrojo
  pinMode(sensorPin1, INPUT);   //definir pin como entrada
  pinMode(sensorPin2, INPUT);  
  pinMode(sensorPin3, INPUT);  
  pinMode(sensorPin4, INPUT);  
  pinMode(sensorPin5, INPUT);  
  pinMode(sensorPin6, INPUT);
  pinMode(sensorPin7, INPUT);
  pinMode(sensorPin8, INPUT); 

  //led
  const int led = 13;
  pinMode(led,OUTPUT);
  //Motores
  MotorL.attach(8);
  MotorR.attach(9);

  //Apaga motores
  MotorL.writeMicroseconds(1500);
  MotorR.writeMicroseconds(1500);

  delay(10000);
  Serial.begin(115200);
  delay(10);
  // Encendemos el motor ajustando la velocidad de 1000 a 2000 (máximo de velocidad de retroceso y avance), siendo 1500 el stop
  
}

//Limpia las variables de la red como net, ft y salidas. También el arreglo de los sensores
void CleanVariables(void){
  int i;
  for(i=0;i<numberNeurons;i++){
    if(i<sensorsNumber){
      sensors[i]=0.0f;
    }
    net[i]=0.0f;
    TF[i]=0.0f;
    output[i]; //Salidas
  }
}

//Lectura del sensor ultrasonico
void UltrasonicSensor(void){
  long t;  //timpo que demora en llegar el eco
  long d;  //distancia en centimetros

  digitalWrite(Trigger1, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger1, LOW);
  t = pulseIn(Echo1, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[8] = 1;
  //delay(100);          //Hacemos una pausa de 100ms
  
  digitalWrite(Trigger2, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger2, LOW);
  t = pulseIn(Echo2, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[9] = 1;
  //delay(100);          //Hacemos una pausa de 100ms
  
  digitalWrite(Trigger3, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger3, LOW);
  t = pulseIn(Echo3, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[10] = 1;
   
  digitalWrite(Trigger4, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger4, LOW);
  t = pulseIn(Echo4, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[11] = 1;
  
  digitalWrite(Trigger5, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger5, LOW);
  t = pulseIn(Echo5, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[12] = 1;

  digitalWrite(Trigger6, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger6, LOW);
  t = pulseIn(Echo6, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[13] = 1;
  
  digitalWrite(Trigger7, HIGH); 
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger7, LOW);
  t = pulseIn(Echo7, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[14] = 1;

  digitalWrite(Trigger8, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger8, LOW);
  t = pulseIn(Echo8, HIGH);  //obtenemos el ancho del pulso
  d = t / 59;               //escalamos el tiempo a una distancia en cm
  if (d < 5) sensors[15] = 1;
}

void IRSensor(void){
  int value = 0, i;
  value = digitalRead(sensorPin1);  //lectura digital de pin
  if (value == LOW)
    sensors[0] = 1;
    
  value = digitalRead(sensorPin2);  //lectura digital de pin
  if (value == LOW)
    sensors[1] = 1;

  value = digitalRead(sensorPin3);  //lectura digital de pin
  if (value == LOW)
    sensors[2] = 1;

  value = digitalRead(sensorPin4);  //lectura digital de pin
  if (value == LOW)
    sensors[3] = 1;
  
  value = digitalRead(sensorPin5);  //lectura digital de pin
  if (value == LOW)
    sensors[4] = 1;
  
  value = digitalRead(sensorPin6);  //lectura digital de pin
  if (value == LOW)
    sensors[5] = 1;

  value = digitalRead(sensorPin7);  //lectura digital de pin
  if (value == LOW) 
    sensors[6] = 1;
    
  value = digitalRead(sensorPin8);  //lectura digital de pin
  if (value == LOW)
    sensors[7] = 1;
}

void SenseStep(void){
  UltrasonicSensor();
  IRSensor();
}

void NetComp(void){
  int i, j, k; //i contador de neurona, j contador de pesos, k contador de entradas
  //Capa de entrada
  j=0;
  for(i=0;i<inputNumber;i++){
    for(k=0;k<sensorsNumber;k++){
      net[i]+=weights[j]*sensors[k];
      j++;
    }
    net[i]+=weights[j];
    j++;
    TF[i]= 1 / (1 + exp(-1 * net[i]));
    if(TF[i]>0.5)
      output[i]=1;
  }
  for(i=inputNumber;i<numberNeurons;i++){
    for(k=0;k<inputNumber;k++){
      net[i]+=weights[j]*output[k];
      j++;
    }
    net[i]+=weights[j];
    j++;
    TF[i]= 1 / (1 + exp(-1 * net[i]));
    if(TF[i]>0.5)
      output[i]=1;
  }
}

void MotorMotion(void){
  int timeMotion =10000;
  int i=inputNumber; //Comienza con la primera de las neuronas de salida
  if(output[i]>0.5) {     //Reversa Fuerte
    MotorL.writeMicroseconds(1100);
    MotorR.writeMicroseconds(1100);
    Serial.println("RF");
    delay(timeMotion/4); 
    MotorL.writeMicroseconds(1100);
    MotorR.writeMicroseconds(1100);
    delay(timeMotion/4); 
    Serial.println("RF");
  }
  else{
    if(output[i+1]>0.5){
      MotorL.writeMicroseconds(1350);
      MotorR.writeMicroseconds(1350);
      Serial.println("RB");
      delay(timeMotion/4); 
      MotorL.writeMicroseconds(1350);
      MotorR.writeMicroseconds(1350);
      Serial.println("RB");
      delay(timeMotion/4); 
    }
    else{
      if(output[i+2]>0.5){
        MotorL.writeMicroseconds(1500);
        MotorR.writeMicroseconds(1500);
        Serial.println("STOP");
        delay(timeMotion/4); 
        MotorL.writeMicroseconds(1500);
        MotorR.writeMicroseconds(1500);
        Serial.println("STOP");
        delay(timeMotion/4); 
      }
      else{
        if(output[i+3]>0.5){
          MotorL.writeMicroseconds(1900);
          MotorR.writeMicroseconds(1900);
          Serial.println("AF");
          delay(timeMotion/4); 
          MotorL.writeMicroseconds(1900);
          MotorR.writeMicroseconds(1900);
          Serial.println("AF");
          delay(timeMotion/4); 
        }
        else{
          if(output[i+4]>0.5){
            MotorL.writeMicroseconds(1650);
            MotorR.writeMicroseconds(1650);
            Serial.println("AB");
            delay(timeMotion/4); 
            MotorL.writeMicroseconds(1650);
            MotorR.writeMicroseconds(1650);
            Serial.println("AB");
            delay(timeMotion/4); 
          }
        }
      }
    }
  }
  MotorL.writeMicroseconds(1600);
  MotorR.writeMicroseconds(1600);
  Serial.println("Detener");
  delay(3000);
  MotorL.writeMicroseconds(1500);
  MotorR.writeMicroseconds(1500);
  delay(3);
}

void loop() {
  CleanVariables();
  SenseStep();
  NetComp();
  MotorMotion();
  delay(500); //espera 500mS  
} 
