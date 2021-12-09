
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xE8E8F0F0E1LL;

RF24 radio(7, 8); // CE, CSN

//Para reducir la potencia de control disminuir la variable 'divider'
//Recibe valores entre 0 y 1
float divider = 0.5;

struct MyData {
  byte roll;
  byte pitch;
  byte yaw;
  byte throttle;
  byte AUX1;
  byte AUX2;
  byte arming_steps;
  byte Rth;
  byte Rpi;
  byte Rro;
  byte Lth;
  byte Lpi;
  byte Lro;
};

MyData data;

void resetData() 
{
  //Valores iniciales de cada canal (Incluyendo la informacion extra
  //para el modulo de retroalimentacion)
  //La aceleracion se mantiene en 0
  //127 es el valor medio de control
    
  data.throttle = 255;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
  data.arming_steps = 0;
  data.Rth = 0;
  data.Rpi = 0;
  data.Rro = 0;
  data.Lth = 0;
  data.Lpi = 0;
  data.Lro = 0;
}

String inputString = "";  // La cadena de texto enviada por unity
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(200); 
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();
}

void loop() {
  if (stringComplete) {
    inputString.trim();
    sendData(inputString);
    inputString = "";
    stringComplete = false;
  }
  
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    }
    else {
      inputString += inChar;
    }
  }
} 

void sendData(String cmd){
  int values[13];
  int indexStart = -1;
  int indexEnd = 0;
  for(int j = 0; j <= 12; j++){
      indexEnd = cmd.indexOf(",", indexStart+1);
      String cmTh = cmd.substring(indexStart+1, indexEnd);
      values[j] = cmTh.toInt();
      indexStart = indexEnd;
    }
    
  data.AUX2     = values[5];
  
  data.throttle = ((values[0])*divider);
  
  if(data.AUX2 >= 203){
    data.throttle = 100;
    }
    
  data.yaw      = ((values[1])*divider)+(127*(1-divider));
  data.pitch    = ((values[2])*divider)+(127*(1-divider));
  data.roll     = ((values[3])*divider)+(127*(1-divider));
  data.AUX1     = values[4];
  data.arming_steps     = values[6];
  data.Rth      = values[7];
  data.Rpi      = values[8];
  data.Rro      = values[9];
  data.Lth      = values[10];
  data.Lpi      = values[11];
  data.Lro      = values[12];
  radio.write(&data, sizeof(MyData));
} 
