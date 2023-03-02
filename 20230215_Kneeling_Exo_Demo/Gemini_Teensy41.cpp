#include "Gemini_Teensy41.h"

Gemini_Teensy41::Gemini_Teensy41(uint32_t id, int Can_id, double Gear_ratio)
{
  ID = id;
  gear_ratio = Gear_ratio;
}
Gemini_Teensy41::~Gemini_Teensy41()
{}

void Gemini_Teensy41::init_motor()
{
//  close_motor();
//  receive_CAN_data();
//  delay(100);
//  stop_motor();
//  receive_CAN_data();
  delay(100);
  send_current_command(0);
  delay(100);
  send_speed_command(0);
  delay(100);
  read_multi_turns_angle_for36();
  delay(100);
  receive_CAN_data();
  delay(100);
  motorAngle_offset = motorAngle_raw;
  delay(100);
}

////////////////Received CAN Message Decoding////////////////////////////////
void Gemini_Teensy41::DataExplanation(CAN_message_t msgR2)
{
  int len = msgR2.len;
  if (len == 8)
  {
    switch (msgR2.buf[0])
    {
      case 0x30://1.Read PID gain
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        break;
      case 0x31: //2.write PID to RAM
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        break;
      case 0x32: //3.write PID to ROM
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        break;
      case 0x33: //4.read Accel
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        break;
      case 0x34: //5.write Accel to RAM
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
        break;
      case 0x90: //6: read encoder (encoder, encoderRaw,encoderOffset)
        encoder = (uint16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        encoderRaw = (uint16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0x91: //7:write certain position as zero position to ROM
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0x19: //8:write current position as zero position to ROM
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0x92: //9: read multi-turn motorAngle (it is int32 datatype ; datasheet is wrong)

        motorAngle_int32 = (int32_t)( ((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4] )) ;
        motorAngle_int32 = motorAngle_int32 * 0.01 ;
        motorAngle_raw = ((double) motorAngle_int32);
        motorAngle = motorAngle_raw - motorAngle_offset;
//        Serial.print(motorAngle_raw);
//        Serial.print("  ");
//Serial.print(ID);
//Serial.print(motorAngle);
//Serial.print(motorAngle_raw);
//Serial.println(motorAngle_offset);
//Serial.println("  ");

        break;
      case 0x94: //10: read single turn circle angle
        circleAngle = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));

        break;
      case 0x95: //11: clear angle command
        break;
      case 0x9A: //12: read motor status temperature, voltage, errorState
        temperature = (int8_t)msgR2.buf[1];
        voltage = (uint16_t)(((uint16_t)msgR2.buf[4] << 8) | ((uint16_t)msgR2.buf[3]));
        errorState = msgR2.buf[7];
        if (errorState & 0x01)
        {
        }
        else if (errorState & 0x08)
        {
        }
        break;
      case 0x9B: //13: clear motor error status return temperature, voltage, errorState
        temperature = (int8_t)msgR2.buf[1];
        voltage = (uint16_t)(((uint16_t)msgR2.buf[4] << 8) | ((uint16_t)msgR2.buf[3]));
        errorState = msgR2.buf[7];
        if (errorState & 0x01)
        {
        }
        else if (errorState & 0x08)
        {
          Serial.println("Over temperature protection");
        }
        break;
      case 0x9C: //14: read motor status 2 return temperature, iq current/100, speed (1dps/LSB), encoder (0~16383)
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq)/100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
                Serial.print("; encoder is ");
        break;
      case 0x9D: //15: read motor status 3 return temperature, phase A B C current
        temperature = (int8_t)msgR2.buf[1];
        iA = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iA_A = ((double)iA) / 100;
        iB = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        iB_A = ((double)iB) / 100;
        iC = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        iC_A = ((double)iC) / 100;
        break;
      case 0x80: //16: close motor and clear all command
        break;
      case 0x81: //17: stop motor and keep all command
        break;
      case 0x88: //18: start motor
        break;
      case 0xA1: //19: send current command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA2: //20: send speed command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA3: //21: send position command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA4: //22: send position command 2(mulitturn command) and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA6: //23: send position command 3(single turn command) and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) / 100;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      default:
        break;
    }
  }
}
/////////////////////////////////////////////////////////////////////

//******1.Read PID data command******//
void Gemini_Teensy41::read_PID()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x30;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******2.Write PID gain to RAM******//
void Gemini_Teensy41::write_PID_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x31;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******3.Write PID gain to ROM******//
void Gemini_Teensy41::write_PID_ROM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x32;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******4.Read Acceleration******//
void Gemini_Teensy41::read_acceleration()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x33;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******5.Write Acceleration RAM******//
void Gemini_Teensy41::write_acceleration_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x34;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&Accel);
  msgW.buf[5] = *((uint8_t*)(&Accel) + 1);
  msgW.buf[6] = *((uint8_t*)(&Accel) + 2);
  msgW.buf[7] = *((uint8_t*)(&Accel) + 3);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******6.Read encoder Position******//
void Gemini_Teensy41::read_encoder()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x90;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //receive_CAN_data();
}
//******7.Write ENCODER OFFSET ROM******//
void Gemini_Teensy41::write_encoder_offset_RAM(uint16_t encoder_Offset )
{
  encoderOffset = encoder_Offset;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x91;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = *(uint8_t*)(&encoderOffset);
  msgW.buf[7] = *((uint8_t*)(&encoderOffset) + 1);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******8.Write current postioton as Zero degree******//
void Gemini_Teensy41::write_current_position_as_zero_position()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x19;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******9.Read multi turns angle command******//
void Gemini_Teensy41::read_multi_turns_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //delay(1);
  //receive_CAN_data();
}
//******10.Read single circle angle command******//
void Gemini_Teensy41::read_single_turns_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x94;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//(current cannot use it)******11.clear all angle command and offset currnet position as zero command (unit 0.01deg/LSB)******//
void Gemini_Teensy41::clear_motor_angle_command()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x95;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******12.read motor status and error (temperature(1degreeC/LSB), voltage(0.1V/LSB), errorState:(0: normal, 1:abnormal)0bit Voltage 3bit temperature) ******//
void Gemini_Teensy41::read_motor_status()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9A;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******13.clear motot error and reset motor******//
void Gemini_Teensy41::clear_motor_error()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9B;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}
//******14.read motor status 2 (temperature 1degreeC/LSB, iq/100, speed(1dps/LSB), 14 bit encoder value(0~16383))******//
void Gemini_Teensy41::read_motor_status_2()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9C;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}
//******15.read motor status 3 (temperature 1degreeC/LSB,A phase current(1A/64LSB),B phase current(1A/64LSB),C phase current(1A/64LSB) )******//
void Gemini_Teensy41::read_motor_status_3()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9D;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}
//******16.close motor and clear all command******//
void Gemini_Teensy41::close_motor()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x80;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}
//******17.stop motor but don't clear any previous command******//
void Gemini_Teensy41::stop_motor()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x81;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}
//******18.start motor******//
void Gemini_Teensy41::start_motor()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0X88;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  if (Can3.write(msgW))
  {
    Serial.println("Succeful to send start motor command");
  }
  else
  {
    Serial.println("Fail to send start motor command");
  }
  //  delay(1);
}
//******19.current control: send current command current unit A(not limited by maximum Torque Current)******//
void Gemini_Teensy41::send_current_command(double current)
{
  current = current * 100;
  iqControl = (int16_t)current;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&iqControl);
  msgW.buf[5] = *((uint8_t*)(&iqControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}
//******20.speed control: send speed command speed unit dps******//
void Gemini_Teensy41::send_speed_command(double speedvalue)
{
  speedvalue = speedvalue * 100 * gear_ratio;
  speedControl = (int32_t)speedvalue;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA2;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&speedControl);
  msgW.buf[5] = *((uint8_t*)(&speedControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&speedControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&speedControl) + 3);
  Can3.write(msgW);
}
//******21.position control:send position command (angle unit degree)******//
void Gemini_Teensy41::send_position_command(double angle)
{
  angle = angle * 100 * gear_ratio;
  angleControl = (int32_t)angle;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA3;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = 0x00;
  msgW.buf[3] = 0x00;
  msgW.buf[4] = ((uint8_t*)(&angleControl))[0];
  msgW.buf[5] = ((uint8_t*)(&angleControl))[1];
  msgW.buf[6] = ((uint8_t*)(&angleControl))[2];
  msgW.buf[7] = ((uint8_t*)(&angleControl))[3];
  Can3.write(msgW);
}
//******22.position control 2:send multi-turns position command (angle unit degree)
void Gemini_Teensy41::send_position_command_2(double angle, double max_speed)
{
  angle = angle * 100;
  angleControl = (int32_t)angle;
  maxiSpeed = (int16_t)max_speed;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA4;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[3] = *((uint8_t*)(&maxiSpeed) + 1);
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&angleControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&angleControl) + 3);
  Can3.write(msgW);
}
//******23.position control 3:send single-turn position command (angle unit degree 0~359.99)
void Gemini_Teensy41::send_position_command_3(double angle, uint8_t spinDirection)
{
  angle = angle * 100;
  angleControl = (uint8_t)angle;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA6;
  msgW.buf[1] = spinDirection;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}
//******24.position control 4:send single-turn position command (angle unit degree 0~359.99)
void Gemini_Teensy41::send_position_command_4(double angle, double max_speed, uint8_t spinDirection)
{
  angle = angle * 100;
  angleControl = (uint8_t)angle;
  maxiSpeed = (uint8_t)max_speed;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA6;
  msgW.buf[1] = spinDirection;
  msgW.buf[2] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[3] = *(uint8_t*)(&maxiSpeed);

  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}
//******25.multi-motors current control
//ID must be #1~#4 for four motor
void Gemini_Teensy41::send_multi_motor_current_command(double Motor1_current, double Motor2_current, double Motor3_current, double Motor4_current)
{
  iqControl_1 = (int16_t)Motor1_current;
  iqControl_2 = (int16_t)Motor2_current;
  iqControl_3 = (int16_t)Motor3_current;
  iqControl_4 = (int16_t)Motor4_current;
  msgW.id = 0x280;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = *(uint8_t *)(&iqControl_1);
  msgW.buf[1] = *((uint8_t *)(&iqControl_1) + 1);
  msgW.buf[2] = *(uint8_t *)(&iqControl_2);
  msgW.buf[3] = *((uint8_t *)(&iqControl_2) + 1);
  msgW.buf[4] = *(uint8_t *)(&iqControl_3);
  msgW.buf[5] = *((uint8_t *)(&iqControl_3) + 1);
  msgW.buf[6] = *(uint8_t *)(&iqControl_4);
  msgW.buf[7] = *((uint8_t *)(&iqControl_4) + 1);
  Can3.write(msgW);
}
void Gemini_Teensy41::receive_CAN_data()
{
    Serial.println(msgR.id);
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    DataExplanation(msgR);
  }
}


void Gemini_Teensy41::send_current_command_for36(double current)
{
  current = current * 100;
  iqControl = (int16_t)current;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&iqControl);
  msgW.buf[5] = *((uint8_t*)(&iqControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}

void Gemini_Teensy41::read_multi_turns_angle_for36()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //delay(1);
  //receive_CAN_data();
}

void Gemini_Teensy41::read_motor_current_for36()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9C;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
    //delay(1);
}
