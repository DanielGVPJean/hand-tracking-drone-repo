
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 6       // DHT-11 Pin de conexion
#define DHTTYPE    DHT11     // DHT11 o DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);

#include <NewPing.h>

#define TRIGGER_PIN  10
#define ECHO_PIN     9 
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float duration, distance, temp, humid, vel_sonido;
byte powerS;

//====================================Radio

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 7); // CE, CSN

//====================================Fin de configuracion de Radio

////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 7  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
int ppm[channel_number];
//////////////////////////////////////////////////////////////////

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

// El tamaño del struct no deberá exceder los 32 bytes
struct MyData {
  byte roll;
  byte pitch;
  byte yaw;
  byte throttle;
  byte AUX1;
  byte AUX2;
  byte arming_steps;
};

MyData data;

void resetData() 
{
  // Valores de seguridad para cuando no se detecta un flujo de informacion entre el transmisor y el receptor
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 204;
  data.AUX2= 204;
  data.arming_steps = 0;
  
  setPPMValuesFromData();
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.roll, 0, 255, 1000, 2000);
  ppm[1] = map(data.pitch,      0, 255, 1000, 2000);
  ppm[2] = map(data.throttle,    0, 255, 1000, 2000);
  ppm[3] = map(data.yaw,     0, 255, 1000, 2000);
  ppm[4] = map(data.AUX1,     0, 255, 1000, 2000);
  ppm[5] = map(data.AUX2,     0, 255, 1000, 2000);
  ppm[6] = map(distance,     0, 200, 1000, 2000);
  
  }
/**************************************************/

void setupPPM() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //Inicializa la señal PPM con su valor predeterminado (Apagado)

  cli();
  TCCR1A = 0; //Inicializa el registro TCCR1 a 0
  TCCR1B = 0;

  OCR1A = 100;  //Compara la coincidencia entre los registros, inicia el limite de tiempo para el primer interrupt
  TCCR1B |= (1 << WGM12);  //Enciende el modo CTC
  TCCR1B |= (1 << CS11);  //8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); //Habilita el temporizador del interrupt
  sei();
}
void setup()
{                

  Serial.begin(9600);
  
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  
  resetData();
  setupPPM();
  
  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS); //El transmisor debe tener el DataRate igual a este.
  radio.setAutoAck(false);
  
  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
}  // End of setup


/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}

/**************************************************/

unsigned long last_DHT_Read = 0;
unsigned long last_SR04_Read = 0;
void loop()  // Start of loop
{
  recvData();
  
  unsigned long now = millis();
  
  if ( now - lastRecvTime > 1000 ) {
    //Si se perdio la señal
    resetData();
  }
  
  if(data.AUX2 >= 203){
    resetData();
    if(now - last_SR04_Read > 50){
      duration = sonar.ping_median(5);
      duration = (duration / 2);
      last_SR04_Read = millis();
      }
    
    if(now - last_DHT_Read > 1000){
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      temp = event.temperature;
      dht.humidity().getEvent(&event);
      humid = event.relative_humidity;
      last_DHT_Read = millis();
      }
      
    vel_sonido = (((sqrt((1.4)*(287.05)*(temp+273.15)))+(0.0124*humid))*100)/1000000;
  
    distance = vel_sonido * duration;
    if(distance >= 200 || distance == 0){
      data.throttle = 50;
      }else{
        powerS = map(distance, 200, 1, 50, 60);
        data.throttle = powerS;
        if(distance < 10){
          data.AUX1 = 0;
        }
      }
  }

  setPPMValuesFromData();

}

#define clockMultiplier 1 //2 es para el arduino a 16MHz y 1 es para el de 8MHz

ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; //Apaga el pin 2. Tambien podria usar digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; //Enciende el pin 2. Tambien podria usar digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
