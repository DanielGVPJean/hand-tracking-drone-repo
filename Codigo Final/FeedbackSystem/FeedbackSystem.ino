#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE, CSN

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

//El tamaño del struct no deberá exceder los 32 bytes
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
  // Valores de seguridad para cuando no se detecta un flujo de informacion entre el transmisor y el receptor
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2= 0;
  data.arming_steps = 0;
  data.Rth = 0;
  data.Rpi = 0;
  data.Rro = 0;
  data.Lth = 0;
  data.Lpi = 0;
  data.Lro = 0;
}
static int m_R_Up_Pin = 4;
static int m_R_Down_Pin = 6;
static int m_R_Left_Pin = 3;
static int m_R_Right_Pin = 5;
//*--------------------------------- Inicio de variables de control para vibradores
bool m_R_Up = false;
bool m_R_Down = false;
bool m_R_Left = false;
bool m_R_Right = false;

unsigned long last_m_R_Up = 0;
unsigned long last_m_R_Down = 0;
unsigned long last_m_R_Left = 0;
unsigned long last_m_R_Right = 0;

int m_R_Up_Freq = 0;
int m_R_Down_Freq = 0;
int m_R_Left_Freq = 0;
int m_R_Right_Freq = 0;

static int m_L_Up_Pin = 8;
static int m_L_Down_Pin = 12;
static int m_L_Left_Pin = 7;
static int m_L_Right_Pin = 11;

bool m_L_Up = false;
bool m_L_Down = false;
bool m_L_Left = false;
bool m_L_Right = false;

unsigned long last_m_L_Up = 0;
unsigned long last_m_L_Down = 0;
unsigned long last_m_L_Left = 0;
unsigned long last_m_L_Right = 0;

int m_L_Up_Freq = 0;
int m_L_Down_Freq = 0;
int m_L_Left_Freq = 0;
int m_L_Right_Freq = 0;
//*--------------------------------- Fin de variables de control para vibradores

int prev_arming_steps = 0;
int vibrate_start_time_R = 0;
int vibrate_start_time_L = 0;
int vibrate_start_time = 0;
int vibrate_times;

int arming_finalized = 0;
int engage_Motors = 0;
int STOP_ALL = 0;

int L_Hand_retro = 0;
byte prev_aux1 = 102;
int R_Hand_retro = 0;
byte prev_aux2 = 102;

void setup() {
  resetData();
  Serial.begin(9600);
  pinMode(m_R_Up_Pin, OUTPUT);
  pinMode(m_R_Down_Pin, OUTPUT);
  pinMode(m_R_Left_Pin, OUTPUT);
  pinMode(m_R_Right_Pin, OUTPUT);
  
  pinMode(m_L_Up_Pin, OUTPUT);
  pinMode(m_L_Down_Pin, OUTPUT);
  pinMode(m_L_Left_Pin, OUTPUT);
  pinMode(m_L_Right_Pin, OUTPUT);
  for(int j = 3; j<=8; j++){
    analogWrite(j, 0);
    }
    analogWrite(11, 0);
    analogWrite(12, 0);
    
  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS); //El transmisor debe tener el DataRate igual a este.
  radio.setAutoAck(false);

  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
}

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

void loop() {
  recvData();

  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
    STOP_ALL = 1;
  }else{
    if(STOP_ALL == 1){
      STOP_ALL = 0;
      turn_Off_All();
    }
  }
  Serial.print(data.arming_steps);
  Serial.print("--En");
  Serial.print(engage_Motors);
  Serial.print("--ArF");
  Serial.print(arming_finalized);
  Serial.print("--VS");
  Serial.print(vibrate_start_time_R);
  Serial.print("--VT");
  Serial.print(vibrate_times);
  Serial.print("--A2");
  Serial.println(data.AUX2);
if(STOP_ALL == 0){
  if(data.arming_steps <= 4 && engage_Motors == 0 && arming_finalized == 0){
      m_R_Down_Freq = 100;
      m_L_Up_Freq = 100;
      if(data.arming_steps == 0){
        prev_arming_steps = 0;
      }else if(data.arming_steps == 1){
          if(prev_arming_steps == 0){
            turn_Off_All();
             vibrate_start_time_R = 0;
             vibrate_start_time_L = 0;
             prev_arming_steps = 1;
             m_R_Down = false;
             m_L_Up = false;
            }
          vibrate_times = 1;
          manageMotor(m_R_Down, last_m_R_Down, m_R_Down_Pin, m_R_Down_Freq, vibrate_start_time_R, vibrate_times);
          manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time_L, vibrate_times);
        }else if(data.arming_steps == 2){
          if(prev_arming_steps == 1){
            turn_Off_All();
             vibrate_start_time_R = 0;
             vibrate_start_time_L = 0;
             prev_arming_steps = 2;
             m_R_Down = false;
             m_L_Up = false;
            }
          vibrate_times = 2;
          manageMotor(m_R_Down, last_m_R_Down, m_R_Down_Pin, m_R_Down_Freq, vibrate_start_time_R, vibrate_times);
          manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time_L, vibrate_times);
        }else if(data.arming_steps == 3){
          if(prev_arming_steps == 2){
            turn_Off_All();
             vibrate_start_time_R = 0;
             vibrate_start_time_L = 0;
             prev_arming_steps = 3;
             m_R_Down = false;
             m_L_Up = false;
            }
          vibrate_times = 3;
          manageMotor(m_R_Down, last_m_R_Down, m_R_Down_Pin, m_R_Down_Freq, vibrate_start_time_R, vibrate_times);
          manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time_L, vibrate_times);
        }else if(data.arming_steps == 4){
          if(prev_arming_steps == 3){
            turn_Off_All();
             vibrate_start_time_R = 0;
             vibrate_start_time_L = 0;
             prev_arming_steps = 4;
             m_R_Up = false;
             m_L_Up = false;
            }
          m_R_Up_Freq = 10;
          m_L_Up_Freq = 10;
          vibrate_times = 6;
          manageMotor(m_R_Up, last_m_R_Up, m_R_Up_Pin, m_R_Up_Freq, vibrate_start_time_R, vibrate_times);
          manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time_L, vibrate_times);
          if(vibrate_start_time_R >= 6){
            turn_Off_All();
            engage_Motors = 1;
            arming_finalized = 1;
          }
        }
    }
    
    if(data.AUX1 != prev_aux1){
      L_Hand_retro = 1;
    }
    if(L_Hand_retro != 0 && arming_finalized == 1){
      if(L_Hand_retro == 1){
        vibrate_start_time_L = 0;
        L_Hand_retro = 2;
      }
        engage_Motors = 0;
        m_L_Up_Freq = 100;
        if(data.AUX1 == 102){
          vibrate_times = 5;
        }else{
          vibrate_times = 3;
        }
        manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time_L, vibrate_times);
        if(vibrate_start_time_L >= vibrate_times){
          engage_Motors = 1;
          L_Hand_retro = 0;
          turn_Off_All();
        }
      }
    prev_aux1 = data.AUX1;
    
    if(data.AUX2 != prev_aux2){
      R_Hand_retro = 1;
    }
    if(R_Hand_retro != 0 && arming_finalized == 1){
      if(R_Hand_retro == 1){
        vibrate_start_time_R = 0;
        R_Hand_retro = 2;
      }
        engage_Motors = 0;
        m_R_Up_Freq = 50;
        m_R_Down_Freq = 50;
        if(data.AUX2 == 102){
          vibrate_times = 5;
          manageMotor(m_R_Up, last_m_R_Up, m_R_Up_Pin, m_R_Up_Freq, vibrate_start_time_R, vibrate_times);
        }else{
          vibrate_times = 5;
          manageMotor(m_R_Down, last_m_R_Down, m_R_Down_Pin, m_R_Down_Freq, vibrate_start_time_R, vibrate_times);
        }
        if(vibrate_start_time_R >= vibrate_times){
          engage_Motors = 1;
          R_Hand_retro = 0;
          turn_Off_All();
        }
      }
    prev_aux2 = data.AUX2;
    
    if(engage_Motors == 1){
      setFrequency();
      vibrate_times = 1;
      vibrate_start_time = 0;
      //Vibradores mano derecha
      vibrate_start_time = 0;
      manageMotor(m_R_Up, last_m_R_Up, m_R_Up_Pin, m_R_Up_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      manageMotor(m_R_Down, last_m_R_Down, m_R_Down_Pin, m_R_Down_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      manageMotor(m_R_Left, last_m_R_Left, m_R_Left_Pin, m_R_Left_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      manageMotor(m_R_Right, last_m_R_Right, m_R_Right_Pin, m_R_Right_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      //Vibradores mano izquierda
      manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      manageMotor(m_L_Down, last_m_L_Down, m_L_Down_Pin, m_L_Down_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      manageMotor(m_L_Left, last_m_L_Left, m_L_Left_Pin, m_L_Left_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
      manageMotor(m_L_Right, last_m_L_Right, m_L_Right_Pin, m_L_Right_Freq, vibrate_start_time, vibrate_times);
      vibrate_start_time = 0;
   }
 }else{
  if(arming_finalized == 1){
    turn_Off_All();
    }
      vibrate_times = 2;
      vibrate_start_time_R = 1;
      vibrate_start_time_L = 1;
      m_R_Up_Freq = 100;
      m_L_Up_Freq = 200;
      manageMotor(m_R_Up, last_m_R_Up, m_R_Up_Pin, m_R_Up_Freq, vibrate_start_time_R, vibrate_times);
      manageMotor(m_L_Up, last_m_L_Up, m_L_Up_Pin, m_L_Up_Freq, vibrate_start_time_L, vibrate_times);
      engage_Motors = 0;
      arming_finalized = 0;
      prev_arming_steps = 0;
  }//End of STOP ALL
}
void manageMotor(bool& motor_State, unsigned long& last_Change, int& motor_Pin, int& motor_Frequency, int& vibrate_start, int& vibrate_no_times){
  if(motor_State){
      if((last_Change + 100) <= millis()){
          analogWrite(motor_Pin, 0);
          last_Change = millis();
          motor_State = false;
        }
    }else{
      if((last_Change + motor_Frequency) <= millis() && motor_Frequency != 0 && vibrate_start < vibrate_no_times){
          analogWrite(motor_Pin, 120);
          last_Change = millis();
          vibrate_start++;
          motor_State = true;
        }
      }
  }
void turn_Off_All(){
  for(int j = 3; j<=8; j++){
    analogWrite(j, 0);
    }
    analogWrite(11, 0);
    analogWrite(12, 0);
}
void setFrequency(){
  //Up motors
    if(data.Rth >= 63){
      m_R_Up_Freq = map(data.Rth, 63, 127, 500, 10);
      }else{
        m_R_Up_Freq = 0;
        }
    if(data.Lth >= 63){
      m_L_Up_Freq = map(data.Lth, 63, 127, 500, 10);
      }else{
        m_L_Up_Freq = 0;
        }
  //Down motors
    if(data.Rpi >= 100){
      m_R_Down_Freq = map(data.Rpi, 100, 127, 500, 10);
      }else if(data.Rpi <= 30){
        m_R_Down_Freq = map(data.Rpi, 30, 0, 500, 10);
        }else{
          m_R_Down_Freq = 0;
          }
    if(data.Lpi >= 100){
      m_L_Down_Freq = map(data.Lpi, 100, 127, 500, 10);
      }else if(data.Lpi <= 30){
        m_L_Down_Freq = map(data.Lpi, 30, 0, 500, 10);
        }else{
          m_L_Down_Freq = 0;
          }
//Side motors
    if(data.Rro >= 100){
      m_R_Right_Freq = map(data.Rro, 100, 127, 500, 10);
      }else if(data.Rro <= 20){
        m_R_Left_Freq = map(data.Rro, 20, 0, 500, 10);
        }else{
          m_R_Right_Freq = 0;
          m_R_Left_Freq = 0;
          }
     if(data.Lro >= 100){
      m_L_Right_Freq = map(data.Lro, 100, 127, 500, 10);
      }else if(data.Lro <= 20){
        m_L_Left_Freq = map(data.Lro, 20, 0, 500, 10);
        }else{
          m_L_Right_Freq = 0;
          m_L_Left_Freq = 0;
          }
  }
