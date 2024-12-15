
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <mcp2515.h>  
#include <SPI.h>  
#include <Servo.h> 
#include <ros.h>
#include <my_robot_controller/MotorControl.h>
#include <my_robot_controller/MotorFeedback.h>

ros::NodeHandle nh;

MCP2515 mcp2515(10);     // Set CS to pin 10
struct can_frame canSent;
struct can_frame canReceived;

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
int can_id = 0;
float p_in[3] = {0.0f,0.0f,0.0f};
float v_in[3] = {0.0f,0.0f,0.0f};
float kp_in[3] = {0.0f,0.0f,0.0f};
float kd_in[3] = {0.0f,0.0f,0.0f};
float t_in[3] = {0.0f,0.0f,0.0f};

// measured values
float p_out[3] = {0.0f,0.0f,0.0f};
float v_out[3] = {0.0f,0.0f,0.0f};
float t_out[3] = {0.0f,0.0f,0.0f};

void motor1_control_callback(const my_robot_controller::MotorControl &msg1) {
  int8_t motor_id1 = msg1.motor_id_in;
  p_in[motor_id1] = msg1.position_in;
  v_in[motor_id1] = msg1.velocity_in;
  t_in[motor_id1] = msg1.torque_in;
  kp_in[motor_id1] = msg1.k_pot_in;
  kd_in[motor_id1] = msg1.k_der_in;
}

void motor2_control_callback(const my_robot_controller::MotorControl &msg2) {
  int8_t motor_id2 = msg2.motor_id_in;
  p_in[motor_id2] = msg2.position_in;
  v_in[motor_id2] = msg2.velocity_in;
  t_in[motor_id2] = msg2.torque_in;
  kp_in[motor_id2] = msg2.k_pot_in;
  kd_in[motor_id2] = msg2.k_der_in;
}

void motor3_control_callback(const my_robot_controller::MotorControl &msg3) {
  int8_t motor_id3 = msg3.motor_id_in;
  p_in[motor_id3] = msg3.position_in;
  v_in[motor_id3] = msg3.velocity_in;
  t_in[motor_id3] = msg3.torque_in;
  kp_in[motor_id3] = msg3.k_pot_in;
  kd_in[motor_id3] = msg3.k_der_in;
}

//Define Suscribers
ros::Subscriber<my_robot_controller::MotorControl> subscriber_motor1("motor1_control", motor1_control_callback);

ros::Subscriber<my_robot_controller::MotorControl> subscriber_motor2("motor2_control", motor2_control_callback);

ros::Subscriber<my_robot_controller::MotorControl> subscriber_motor3("motor3_control", motor3_control_callback);

// Create instances of the message
my_robot_controller::MotorFeedback motor1_feedback;
my_robot_controller::MotorFeedback motor2_feedback;
my_robot_controller::MotorFeedback motor3_feedback;

// Create individual publishers for each motor
ros::Publisher publisher_motor1_feedback("motor1_feedback", &motor1_feedback);
ros::Publisher publisher_motor2_feedback("motor2_feedback", &motor2_feedback);
ros::Publisher publisher_motor3_feedback("motor3_feedback", &motor3_feedback);

void setup(){

  nh.initNode();

  Serial.begin(115200);
  delay(1000);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("MCP2515 Initialized Successfully!");

  Serial.println("CAN BUS Shield init ok!");

  for (int i=0;i<3;i++){
    delay(5);
    pack_cmd(i+1);

    //receive msg
    delay(5);
    if(mcp2515.readMessage(&canReceived) == MCP2515::ERROR_OK){
      unpack_reply();
    }

    p_in[i]=p_out[i];

  }

  //enable motors
  for (int i=0;i<3;i++){
    delay(500);
    EnterMotorMode(i+1);
  }

  // Initialize publishers for each motor
  nh.advertise(publisher_motor1_feedback);
  nh.advertise(publisher_motor2_feedback);
  nh.advertise(publisher_motor3_feedback);

  nh.subscribe(subscriber_motor1);
  nh.subscribe(subscriber_motor2);
  nh.subscribe(subscriber_motor3);
  
  
}

void loop(){

  for (int i=0;i<3;i++){
    pack_cmd(i+1);
  }

  if (mcp2515.readMessage(&canReceived) == MCP2515::ERROR_OK){
    unpack_reply();
   }

// Populate and publish the feedback for each motor
  motor1_feedback.motor_id_out = 1;
  motor1_feedback.position_out = p_out[0]; 
  motor1_feedback.velocity_out = v_out[0]; 
  motor1_feedback.torque_out = t_out[0];
  publisher_motor1_feedback.publish(&motor1_feedback);  

  motor2_feedback.motor_id_out = 2;
  motor2_feedback.position_out = p_out[1]; 
  motor2_feedback.velocity_out = v_out[1]; 
  motor2_feedback.torque_out = t_out[1];
  publisher_motor2_feedback.publish(&motor2_feedback);

  motor3_feedback.motor_id_out = 3;
  motor3_feedback.position_out = p_out[2]; 
  motor3_feedback.velocity_out = v_out[2]; 
  motor3_feedback.torque_out = t_out[2];
  publisher_motor3_feedback.publish(&motor3_feedback); 

  nh.spinOnce();
  delay(1);
}

void EnterMotorMode(int id){
  struct can_frame canMsg1;
  // Enter Motor Mode (enable)
  canMsg1.can_id = float_to_uint(id,0,3,12);
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xFF;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xFF;
  canMsg1.data[3] = 0xFF;
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = 0xFC;

  mcp2515.sendMessage(&canMsg1);
}

void ExitMotorMode(int id){
  struct can_frame canMsg2;
  // Enter Motor Mode (enable)
  canMsg2.can_id = float_to_uint(id,0,3,12);
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0xFF;
  canMsg2.data[1] = 0xFF;
  canMsg2.data[2] = 0xFF;
  canMsg2.data[3] = 0xFF;
  canMsg2.data[4] = 0xFF;
  canMsg2.data[5] = 0xFF;
  canMsg2.data[6] = 0xFF;
  canMsg2.data[7] = 0xFD;

  mcp2515.sendMessage(&canMsg2);

}

void pack_cmd(int id){
  //byte buf[8];

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
  canSent.can_id = float_to_uint(id,0,3,12);
  canSent.can_dlc = 8;
  canSent.data[0] = p_int >> 8;  
  canSent.data[1] = p_int & 0xFF;
  canSent.data[2] = v_int >> 4;
  canSent.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  canSent.data[4] = kp_int & 0xFF;
  canSent.data[5] = kd_int >> 4;
  canSent.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  canSent.data[7] = t_int & 0xFF;

  mcp2515.sendMessage(&canSent);
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

  unsigned int canId=canReceived.can_id;

  /// unpack ints from can buffer ///
  unsigned int id = canReceived.data[0];
  unsigned int p_int = (canReceived.data[1] << 8) | canReceived.data[2];
  unsigned int v_int = (canReceived.data[3] << 4) | (canReceived.data[4] >> 4);
  unsigned int i_int = ((canReceived.data[4] & 0xF) << 8) | canReceived.data[5];
  /// convert uints to floats ///
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
