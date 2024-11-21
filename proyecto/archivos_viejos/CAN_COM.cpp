#include <mcp_can.h>
#include <SPI.h>

//for timing of the loop:
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
int loopTime;

//joystick:
int Joy1UD = A6;
int Joy2UD = A5;
int Joy2LR = A4;
int Joy3UD = A3;
int Joy3LR = A2;
int Joy4UD = A1;
int Joy4LR = A0;

//Values for the joystick
int chN[7];

//for MIT controllers
// Value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Set values
float p_in[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float v_in[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float kp_in[7] = {40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f};
float kd_in[7] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
float t_in[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// measured values
float p_out[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float v_out[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float t_out[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// limits
float p_in_min[7] = {-1.6f, -1.6f, -1.6f, -1.6f, -1.6f, -1.6f, -1.6f};
float p_in_max[7] = { 1.6f,  1.6f,  1.6f,  1.6f,  1.6f,  1.6f,  1.6f};
float kd_in_norm[7] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
float kd_in_lim[7] = {3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f};

// variable to receive the message
unsigned char len = 0;
// variable to send the message
unsigned char buf[8];

//for MIT controllers
float p_step = 200;

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 3;

MCP_CAN CAN(SPI_CS_PIN);

void setup() 
{
    delay(1000);
    SerialUSB.begin(115200);
    delay(10);
    SerialUSB.println("Let's go!");
    while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 500k
    {
        SerialUSB.println("CAN BUS Shield init fail");
        SerialUSB.println(" Init CAN BUS Shield again");
        delay(100);
    }
    SerialUSB.println("CAN BUS Shield init ok!");
    delay(3000);
    
    //read initial position
    //for each motor
    for (int i = 0; i < 7; i++){
      delay(5);
      pack_cmd(i+1);
      //receive message
      delay(5);
      if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
      {
        unpack_reply();
      }
      p_in[i] = p_out[i];
      SerialUSB.print("Position ");
      SerialUSB.println(p_in[i]);
    }
    
    //enable MIT controllers
    for (int i = 0; i < 7; i++){
      delay(500);
      EnterMotorMode(i+1);
    }
    //LED ON (D6)
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH);   
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event
    loopTime = currentMillis - previousMillis;
    previousMillis = currentMillis;
    //SerialUSB.println(loopTime);
    analogReadResolution(10);

    chN[0] = analogRead(Joy1UD);
    chN[1] = analogRead(Joy2LR);
    chN[2] = analogRead(Joy2UD);
    chN[3] = analogRead(Joy3LR);
    chN[4] = analogRead(Joy3UD);
    chN[5] = analogRead(Joy4LR);
    chN[6] = analogRead(Joy4UD);

    //for each axis
    for (int i = 0; i < 7; i++){
      if (chN[i] > 600) {
        int a;
        a = map(chN[i], 600, 750, 0, p_step);
        p_in[i] = p_in[i] + a/10000.0;
        p_in[i] = constrain(p_in[i], p_in_min[i], p_in_max[i]);
        if (p_in[i] > 0.95*p_in_max[i]) {
          kd_in[i] = (kd_in_lim[i]-kd_in_norm[i])*(p_in[i]/p_in_max[i]-0.95)/0.05+kd_in_norm[i];
        } else {
          kd_in[i] = kd_in_norm[i];
        }
      }
      if (chN[i] < 400) {
        int a;
        a = map(chN[i], 400, 250, 0, -p_step);
        p_in[i] =  p_in[i] + a/10000.0;
        p_in[i] = constrain(p_in[i], p_in_min[i], p_in_max[i]);
        if (p_in[i] < 0.95*p_in_min[i]) {
          kd_in[i] = (kd_in_lim[i]-kd_in_norm[i])*(p_in[i]/p_in_min[i]-0.95)/0.05+kd_in_norm[i];
        } else {
          kd_in[i] = kd_in_norm[i];
        }
      }
      // send CAN
      pack_cmd(i+1);
      // receive CAN
      if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
      {
        unpack_reply();
      }
    }
    
  }
}

void EnterMotorMode(int id){
  // Enter Motor Mode (enable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;
  CAN.sendMsgBuf(id, 0, 8, buf);
}

void ExitMotorMode(int id){
  // Enter Motor Mode (enable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFD;
  CAN.sendMsgBuf(id, 0, 8, buf);
}

void Zero(int id){
  // Enter Motor Mode (enable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFE;
  CAN.sendMsgBuf(id, 0, 8, buf);
}

void pack_cmd(int id){
  byte buf[8];

  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and + 30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

  /// limit data to be within bounds ///
  float p_des = constrain(p_in[id-1], P_MIN, P_MAX); //fminf(fmaxf(P_MIN, p_in), P_MAX);                    
  float v_des = constrain(v_in[id-1], V_MIN, V_MAX); //fminf(fmaxf(V_MIN, v_in), V_MAX);
  float kp = constrain(kp_in[id-1], KP_MIN, KP_MAX); //fminf(fmaxf(KP_MIN, kp_in), KP_MAX);
  float kd = constrain(kd_in[id-1], KD_MIN, KD_MAX); //fminf(fmaxf(KD_MIN, kd_in), KD_MAX);
  float t_ff = constrain(t_in[id-1], T_MIN, T_MAX); //fminf(fmaxf(T_MIN, t_in), T_MAX);
  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the can buffer ///

  buf[0] = p_int >> 8;                                       
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  CAN.sendMsgBuf(id, 0, 8, buf);
}
 
void unpack_reply(){

  /// CAN Reply Packet Structure ///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and + 30 rad/s
  /// 12 bit current, between -40 and 40;
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows.  For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]] 
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]

  byte len = 0;
  byte buf[8];
  CAN.readMsgBuf(&len, buf);

  unsigned long canId = CAN.getCanId();

  /// unpack ints from can buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  /// convert uints to floats ///
  //SERIAL.println("one");
  p_out[id-1] = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out[id-1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out[id-1] = uint_to_float(i_int, -T_MAX, T_MAX, 12);
} 

unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  }
  if (bits == 16){
    pgg = (unsigned int) ((x-offset)*65535.0/span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if (bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}

// END FILE