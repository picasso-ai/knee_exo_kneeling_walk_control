//1
#include <SPI.h>
#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include "Wireless_IMU.h"
#include <Arduino.h>

int stopFlag = 0;

// CAN_message_t msgR;
/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

int assist_mode = 1;

double weight = 70; // [kg] weight of the subject
uint32_t ID_offset = 0x140;
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, loadcell: port 1, positive current = flexion; left leg flexion encode positive // 36:1 exo updated on 2023-02-10
uint32_t Motor_ID2 = 2; // Motor Can Bus ID, right leg, loadcell: port 2, positive current = extension; right leg flexion encode negative // 36:1 exo updated on 2023-02-10
int CAN_ID = 3;         // CAN port from Teensy
double Gear_ratio = 36;  //The actuator gear ratio, will enfluence actuator angle and angular velocity
double torque_constant = 6.92 * 0.707; // after gear
int Stop_button = 0;    // Stop function
String mode = "start";
double milli_time_current_double = 0;
Gemini_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio);
Gemini_Teensy41 m2(Motor_ID2, CAN_ID, Gear_ratio);
IMU imu;                                                      //Create IMU object see Wireless_IMU.h

double Fsample = 1000;        // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus)
double Fsample_ble = 100;    // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                                        // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                                    // used to control the Bluetooth communication frequency
unsigned long Tinterval_microsecond = (unsigned long)(1000000 / Fsample);               // used to control the teensy controller frequency
unsigned long Tinterval_ble_microsecond = (unsigned long)(1000000 / Fsample_ble);       // used to control the Bluetooth communication frequency

double Cur_command_L = 0;
double Cur_command_R = 0;
int current_limitation = 10; //(unit Amp)//double Tor_command_L = 0;

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy
int LK_ble = 0;                //left knee angle
int RK_ble = 0;                //right knee angle
int current_command_L_ble = 0; //current reference(A) for inner loop current control
int current_command_R_ble = 0;
int current_actual_L_ble = 0;
int current_actual_R_ble = 0;
int torque_command_L_ble = 0; //total torque reference(N-m)
int torque_command_R_ble = 0;
int torque_command_imp_L_ble = 0; // impedance torque
int torque_command_imp_R_ble = 0; // impedance torque
int torque_command_ff_L_ble = 0;  // feedforward torque
int torque_command_ff_R_ble = 0;  // feedforward torque
int torque_L_ble = 0;             //actual torque(N-m)(measured by torque sensor)
int torque_R_ble = 0;
int motor_speed_L_ble = 0;
int motor_speed_R_ble = 0;
double torque_command_L = 0;
double torque_command_R = 0;

//***********High Level Communication*********//
int LK_highlevel = 0;
int RK_highlevel = 0;

int TK_highlevel = 0;
int LT_highlevel = 0;
int RT_highlevel = 0;
int LS_highlevel = 0;
int RS_highlevel = 0;

int Left_Knee_Torque = 0;
int Right_Knee_Torque = 0;
char data_serial_highlevel[101] = {0};
char data_highlevel_rx[7] = {0}; //data high level communication
char Highlevel_Data_Length_Send = 101;
double Left_Knee_Torque_Command;
double Right_Knee_Torque_Command;
double relTime = 0.0;
int squat_bio[140] = { -16,  -16,  -16,  -16,  -15,  -14,  -12,  -11,  -10,  -9, -8, -6, -4, -2, -1, 1,  3,  4,  5,  8,  9,  10, 11, 13, 14, 16, 17, 18, 20, 22, 23, 25, 26, 28, 29, 30, 31, 32, 33, 36, 37, 38, 39, 41, 42, 43, 44, 46, 47, 48, 48, 50, 51, 51, 52, 54, 54, 56, 56, 58, 58, 59, 60, 61, 62, 63, 64, 65, 66, 66, 67, 68, 68, 69, 69, 70, 71, 72, 72, 73, 74, 75, 76, 77, 78, 79, 79, 80, 81, 82, 82, 83, 84, 84, 85, 85, 86, 87, 88, 88, 89, 89, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 91, 92, 92, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93};
int squat_bio_angle = 0;

void setup()
{
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(115200);  //used for communication with computer.
  Serial5.begin(115200); //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  delay(500);
  imu.Gain_E = 6;         //Extension gain for delay output feedback control
  imu.Gain_F = 6;         //Flexion gain for delay output feedback control  DOFC.Gain_E = 10;            //Extension gain for delay output feedback control
  imu.delaypoint = 0;     //realative to delay time (delaypoint*sampletime=delaytime) for delay output feedback control
  reset_motor_angle();
  delay(1000);
  m1.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(1000);
  m2.init_motor(); // Strat the CAN bus communication & Motor Control
  delay(1000);

  CurrentControlSetup();
}

void loop()
{
  CurrentControl();
}

void CurrentControlSetup()
{
  imu.INIT(); //Initialize IMU;
  delay(500);
  imu.INIT_MEAN();
  current_time = micros();
  previous_time = current_time;
  previous_time_ble = current_time;
}

void CurrentControl()
{
  ////******IMU+Current Control Example Torque Constant 0.6 Nm/A**********////////
  imu.READ();                          //Check if IMU data available and read it. the sample rate is 100 hz
  current_time = micros();             //query current time (microsencond)

  //********* use to control the teensy controller frequency **********//
  if (current_time - previous_time > Tinterval_microsecond) // check if the time period of control loop is already larger than Sample period of control loop (Tinterval_microsecond)
  {
    if (Stop_button) //stop
    {
      Cur_command_L = 0;
      Cur_command_R = 0;
    }
    else
    {
      Compute_Cur_Commands();
    }
    Cur_limitation();

    m1.send_current_command(0);//Cur_command_L);
    receive_CAN_data();
    m2.send_current_command(0);
    receive_CAN_data();
    //delay(1);
    for(int qwer=0;qwer<10000;qwer++)
    {
          for(int qwert=0;qwert<10000;qwert++)
    {}
    }
    /////*********Print motor Position - Use below code to read Position 2021-08-17 by Howard*********/////
    m1.read_multi_turns_angle_for36(); //read angle and angle velocity
    receive_CAN_data();
    m2.read_multi_turns_angle_for36(); //read angle and angle velocity
    receive_CAN_data();
    
    previous_time = current_time; //reset previous control loop time
    relTime += Tinterval_microsecond / 1000;
  }

  //********* use to control the Bluetooth communication frequency **********//
  if (current_time - previous_time_ble > Tinterval_ble_microsecond)
  {
    receive_ble_Data();
    send_ble_Data(); // send the BLE data
    previous_time_ble = current_time;
    for (int qqq = 0; qqq < 40; qqq++)
    {
      if (qqq == 5)
      {
        plot_data();
      }
    }
  }
}

void Compute_Cur_Commands() //All commands are in amps
{
  if (assist_mode == 1) //IMU walking
  {
    mode = "Walking (IMU)";

    Cur_command_L = -imu.DOTC[0] / torque_constant; //this is for bilateral walking assistance_left leg
    Cur_command_R = +imu.DOTC[1] / torque_constant; //this is for bilateral walking assistance_right leg
  }
  else if (assist_mode == 2) //sine wave
  {
    mode = "Sine Wave";

    Cur_command_L = 3 * sin(2 * PI * current_time / 1000000) / torque_constant; //(unit Amp);
    Cur_command_R = 3 * sin(2 * PI * current_time / 1000000) / torque_constant; //(unit Amp);
    milli_time_current_double = millis();
    milli_time_current_double = milli_time_current_double / 1000.0;
  }
    else if (assist_mode == 3)
  {
    mode = "Squatting (gravity)";

    Cur_command_L = -0.5 * 64 * sin((m1.motorAngle - m2.motorAngle) / 2 * 3.14 / 180 / 2) / torque_constant; //Amps
    Cur_command_R = 0.5 * 64 * sin((m1.motorAngle - m2.motorAngle) / 2 * 3.14 / 180 / 2) / torque_constant; //Amps
  }
  else if (assist_mode == 4)
  {
    //Change the command value accordingly start with 0.5 increments up to 5.
    mode = "Constant Signal";

    Cur_command_L = -0; //Amps
    Cur_command_R = 0; //Amps
  }
  else if (assist_mode == 5)
  {
    mode = "High Level Control";

    send_serial_Data_Highlevel(); //Serial communication with High level system
    receive_serial_Data_Highlevel();
    Cur_command_L = Left_Knee_Torque_Command / torque_constant;
    Cur_command_R = Right_Knee_Torque_Command / torque_constant;
  }
  else if (assist_mode == 100)
  {
    mode = "Stop";
    Cur_command_L = 0;
    Cur_command_R = 0;
  }
}

void Cur_limitation()
{
  //************* Current limitation *************//

  Cur_command_L = min(current_limitation, Cur_command_L);
  Cur_command_L = max(-current_limitation, Cur_command_L);
  Cur_command_R = min(current_limitation, Cur_command_R);
  Cur_command_R = max(-current_limitation, Cur_command_R);

  torque_command_L = Cur_command_L * torque_constant;
  torque_command_R = Cur_command_R * torque_constant;
}

void receive_ble_Data()
{
  if (Serial5.available() >= 20)
  {
    // Serial.println("-------------New data received-------------------");
    data_rs232_rx[0] = Serial5.read();
    if (data_rs232_rx[0] == 165)
    {
      data_rs232_rx[1] = Serial5.read();
      if (data_rs232_rx[1] == 90)
      {
        data_rs232_rx[2] = Serial5.read();
        if (data_rs232_rx[2] == 20)
        {
          Serial5.readBytes(&data_rs232_rx[3], 17);
          if (data_rs232_rx[3] == 0)
          {
            Stop_button = int(data_rs232_rx[4]);
            if (Stop_button)
            {
              Serial.println("STOP button pressed");
            }
            else
            {
              Serial.println("START button pressed");
            }
          }
          else if (data_rs232_rx[3] == 1)
          {
            assist_mode = int(data_rs232_rx[4]);
          }
          else if (data_rs232_rx[3] == 2)
          {
            float Gain_E = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.Gain_E = Gain_E;
            Serial.print("Extension gain from matlab: ");
            Serial.println(Gain_E);
          }
          else if (data_rs232_rx[3] == 3)
          {
            float Gain_F = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            imu.Gain_F = Gain_F;
            Serial.print("Flexion gain from matlab: ");
            Serial.println(Gain_F);
          }
          else if (data_rs232_rx[3] == 4)
          {
            int delaypoint = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000;

            imu.delaypoint = delaypoint;
            Serial.print("Delay [ms]: ");
            Serial.println(delaypoint * 10);
          }
          else if (data_rs232_rx[3] == 5)
          {
            weight = ((int16_t)(((uint16_t)data_rs232_rx[4]) | ((uint16_t)data_rs232_rx[5] << 8))) / 1000.0;
            Serial.print("Weight [kg]: ");
            Serial.println(weight);
          }
          else if (data_rs232_rx[3] == 6)
          {
          }
          else if (data_rs232_rx[3] == 7)
          {
            reset_motor_angle();
            Serial.println("The angle of motor has been reset");
            imu.INIT_MEAN();
            Serial.println("The angle of IMUs has been reset");
          }
        }
      }
    }
  }
}

void send_ble_Data()
{
  LK_ble = imu.LKx * 100;
  RK_ble = imu.RKx * 100;

  current_command_L_ble = -Cur_command_L * 100; // Gui flexion is negative
  current_command_R_ble = Cur_command_R * 100;  // Gui flexion is negative

  torque_command_L_ble = torque_command_L * 100; // Gui flexion is negative
  torque_L_ble = torque_command_L_ble;  // motor torque constant = 0.232 Nm/A, gear ratio = 9;

  torque_command_R_ble = -torque_command_R * 100; // Gui flexion is negative
  torque_R_ble = torque_command_R_ble;
  //
  motor_speed_L_ble = m1.speed_value / 180 * 3.1415 * 100; // radian
  motor_speed_R_ble = m2.speed_value / 180 * 3.1415 * 100; // radian

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0] = 165;
  data_ble[1] = 90;
  data_ble[2] = datalength_ble;
  data_ble[3] = LK_ble;
  data_ble[4] = LK_ble >> 8;
  data_ble[5] = RK_ble;
  data_ble[6] = RK_ble >> 8;
  data_ble[7] = current_command_L_ble;
  data_ble[8] = current_command_L_ble >> 8;
  data_ble[9] = current_command_R_ble;
  data_ble[10] = current_command_R_ble >> 8;
  data_ble[11] = torque_command_L_ble;
  data_ble[12] = torque_command_L_ble >> 8;
  data_ble[13] = torque_command_R_ble;
  data_ble[14] = torque_command_R_ble >> 8;
  data_ble[15] = torque_L_ble;
  data_ble[16] = torque_L_ble >> 8;
  data_ble[17] = torque_R_ble;
  data_ble[18] = torque_R_ble >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0;
  data_ble[21] = 0;
  data_ble[22] = 0;
  data_ble[23] = 0;
  data_ble[24] = 0;
  data_ble[25] = motor_speed_L_ble;
  data_ble[26] = motor_speed_L_ble >> 8;
  data_ble[27] = motor_speed_R_ble;
  data_ble[28] = motor_speed_R_ble >> 8;
  Serial5.write(data_ble, datalength_ble);
}

//******************Receive high level controller Command****************//
void receive_serial_Data_Highlevel()
{
  if (Serial.available() >= 7)
  {
    //Serial.println("receive");
    data_highlevel_rx[0] = Serial.read();
    if (data_highlevel_rx[0] == 165)
    {
      data_highlevel_rx[1] = Serial.read();
      if (data_highlevel_rx[1] == 90)
      {
        Serial.readBytes(&data_highlevel_rx[2], 5); //int data_rs232_rx[7]
        //Highlevel_Data_Length_Receive = int(data_highlevel_rx[2]); // read data length
        Left_Knee_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[3]) | ((int16_t)data_highlevel_rx[4] << 8)));
        Right_Knee_Torque_Command = ((int16_t)(((int16_t)data_highlevel_rx[5]) | ((int16_t)data_highlevel_rx[6] << 8)));
        Left_Knee_Torque_Command = Left_Knee_Torque_Command / 100;
        Right_Knee_Torque_Command = Right_Knee_Torque_Command / 100;
      }
    }
  }
}
//******************Send high level controller Command****************//
void send_serial_Data_Highlevel()
{
  LK_highlevel = imu.LKx * 100;
  RK_highlevel = imu.RKx * 100;

  TK_highlevel = imu.TKx * 100;
  LT_highlevel = imu.LTx * 100;
  RT_highlevel = imu.RTx * 100;
  LS_highlevel = imu.LSx * 100;
  RS_highlevel = imu.RSx * 100;

  data_serial_highlevel[0] = 165;
  data_serial_highlevel[1] = 90;
  data_serial_highlevel[2] = Highlevel_Data_Length_Send;
  data_serial_highlevel[3] = Left_Knee_Torque >> 8;
  data_serial_highlevel[4] = Left_Knee_Torque;
  data_serial_highlevel[5] = Right_Knee_Torque >> 8;
  data_serial_highlevel[6] = Right_Knee_Torque;
  data_serial_highlevel[7] = LK_highlevel >> 8;
  data_serial_highlevel[8] = LK_highlevel;
  data_serial_highlevel[9] = RK_highlevel >> 8;
  data_serial_highlevel[10] = RK_highlevel;

  data_serial_highlevel[11] = LT_highlevel >> 8;
  data_serial_highlevel[12] = LT_highlevel;
  data_serial_highlevel[29] = RT_highlevel >> 8;
  data_serial_highlevel[30] = RT_highlevel;
  data_serial_highlevel[47] = LS_highlevel >> 8;
  data_serial_highlevel[48] = LS_highlevel;
  data_serial_highlevel[65] = RS_highlevel >> 8;
  data_serial_highlevel[66] = RS_highlevel;
  data_serial_highlevel[83] = TK_highlevel >> 8;
  data_serial_highlevel[84] = TK_highlevel;

  Serial.write(data_serial_highlevel, Highlevel_Data_Length_Send);
}

//**************Plot Data*****************//

void plot_data()
{
  //  Serial.print(Cur_command_L * torque_constant);
  //  Serial.print(" ");
//  Serial.println(imu.LTx);

//  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.RTx);
  Serial.print(" ");
  Serial.print(imu.LSx);
  Serial.print(" ");
  Serial.print(imu.RSx);
  Serial.print(" ");
  Serial.print(Cur_command_L * torque_constant);
  Serial.print(" ");
  Serial.print(Cur_command_R * torque_constant);
  Serial.print(" ");
  Serial.print(m1.motorAngle_offset);
  Serial.print(" ");
  Serial.print(m2.motorAngle_offset);
  Serial.print(" ");
  Serial.print(m1.motorAngle);
  Serial.print(" ");
  Serial.println(m2.motorAngle);
}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}
void receive_CAN_data()
{
  Can3.read(msgR);

  if (msgR.id == (576 + Motor_ID1))//Reply: 0x240 + ID (1~32)
  {
    m1.DataExplanation(msgR);
    //Serial.println("4");
  }
  if (msgR.id == (576 + Motor_ID2))
  {
    m2.DataExplanation(msgR);
    //Serial.println("5");
  }
}

void reset_motor_angle()
{
  delay(1);
  m1.read_multi_turns_angle_for36();
        delay(1);
  receive_CAN_data();
        delay(1);
  m1.motorAngle_offset = m1.motorAngle_raw;
        delay(1);
  m2.read_multi_turns_angle_for36();
        delay(1);
  receive_CAN_data();
        delay(1);
  m2.motorAngle_offset = m2.motorAngle_raw;
        delay(1);
}
