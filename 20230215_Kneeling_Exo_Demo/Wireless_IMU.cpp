#include "Wireless_IMU.h"
union u_tag
{
  byte b[4];
  float fval;
} u;
float IMUdata[52] = {0};

void IMU::Packet_Decode(uint8_t c)
{
  switch (st)
  {
    case 0: // Read 1st Byte
      if (c == 0x3a)
      {
        st = 1;
        Datain[read_count] = c;
        read_count += 1;
      }
      break;
    case 1: // Read 2nd Byte
      if (c == 0xc4)
      {
        st = 2;
        Datain[read_count] = c;
        read_count += 1;
      }
      else
      {
        st = 0;
        read_count = 0;
      }
      break;
    case 2:
      Datain[read_count] = c;
      read_count += 1;
      if (read_count >= 203)
      {
        st = 0;

        GetData();
        TKx = IMUdata[4] - init_TKx; // Trunk angle
        TKy = IMUdata[5] - init_TKy;
        TKz = IMUdata[6] - init_TKz;
        TKAVx = IMUdata[1];
        TKAVy = IMUdata[2];
        TKAVz = IMUdata[3];

        LTx = IMUdata[11] - init_LTx; // Left Thigh
        LTy = IMUdata[12] - init_LTy;
        LTz = IMUdata[13] - init_LTz;
        LTAVx = IMUdata[8];
        LTAVy = IMUdata[9];
        LTAVz = IMUdata[10];

        RTx = IMUdata[18] - init_RTx; // Right Thigh
        RTy = IMUdata[19] - init_RTy;
        RTz = IMUdata[20] - init_RTz;
        RTAVx = IMUdata[15];
        RTAVy = IMUdata[16];
        RTAVz = IMUdata[17];

        LSx = IMUdata[25] - init_LSx; // Left Shank
        LSy = IMUdata[26] - init_LSy;
        LSz = IMUdata[27] - init_LSz;
        LSAVx = IMUdata[22];
        LSAVy = IMUdata[23];
        LSAVz = IMUdata[24];

        RSx = IMUdata[32] - init_RSx; // Right Shank
        RSy = IMUdata[33] - init_RSy;
        RSz = IMUdata[34] - init_RSz;
        RSAVx = IMUdata[29];
        RSAVy = IMUdata[30];
        RSAVz = IMUdata[31];

        LFx = IMUdata[39] - init_LFx; // Left Foot
        LFy = IMUdata[40] - init_LFy;
        LFz = IMUdata[41] - init_LFz;
        LFAVx = IMUdata[36];
        LFAVy = IMUdata[37];
        LFAVz = IMUdata[38];

        RFx = IMUdata[46] - init_RFx; // Right Foot
        RFy = IMUdata[47] - init_RFy;
        RFz = IMUdata[48] - init_RFz;
        RFAVx = IMUdata[43];
        RFAVy = IMUdata[44];
        RFAVz = IMUdata[45];


        DelayOutputTorqueCommand();
        SquatTorqueCommand();
        read_count = 0;
      }
      break;
    default:
      st = 0;
      break;
  }
}

void IMU::INIT()
{
  SERIAL_Wireless.begin(230400);
}

void IMU::READ()
{
  if (SERIAL_Wireless.available())
  {
    ch = SERIAL_Wireless.read();
    Packet_Decode(ch);
  }
}

void IMU::INIT_MEAN()
{
  unsigned long timing = micros();
  while (micros() - timing < INIT_TIME * 1000000)
  {
    unsigned long n = 0;
    READ();
    GetData();

    init_TKx += (IMUdata[4] - init_TKx) / (n + 1);
    init_TKy += (IMUdata[5] - init_TKy) / (n + 1);
    init_TKz += (IMUdata[6] - init_TKz) / (n + 1);

    init_LTx += (IMUdata[11] - init_LTx) / (n + 1);
    init_LTy += (IMUdata[12] - init_LTy) / (n + 1);
    init_LTz += (IMUdata[13] - init_LTz) / (n + 1);

    init_RTx += (IMUdata[18] - init_RTx) / (n + 1);
    init_RTy += (IMUdata[19] - init_RTy) / (n + 1);
    init_RTz += (IMUdata[20] - init_RTz) / (n + 1);

    init_LSx += (IMUdata[25] - init_LSx) / (n + 1);
    init_LSy += (IMUdata[26] - init_LSy) / (n + 1);
    init_LSz += (IMUdata[27] - init_LSz) / (n + 1);

    init_RSx += (IMUdata[32] - init_RSx) / (n + 1);
    init_RSy += (IMUdata[33] - init_RSy) / (n + 1);
    init_RSz += (IMUdata[34] - init_RSz) / (n + 1);

    init_LFx += (IMUdata[39] - init_LFx) / (n + 1);
    init_LFy += (IMUdata[40] - init_LFy) / (n + 1);
    init_LFz += (IMUdata[41] - init_LFz) / (n + 1);

    init_RFx += (IMUdata[46] - init_RFx) / (n + 1);
    init_RFy += (IMUdata[47] - init_RFy) / (n + 1);
    init_RFz += (IMUdata[48] - init_RFz) / (n + 1);

    n += 1;
  }
}

void IMU::GetData()
{
  for (int i = 3; i < 198; i = i + 4)
  {
    u.b[0] = Datain[i];
    u.b[1] = Datain[i + 1];
    u.b[2] = Datain[i + 2];
    u.b[3] = Datain[i + 3];
    IMUdata[(i - 3) / 4] = u.fval;
  }
}

void IMU::DelayOutputTorqueCommand()
{

  // LTx_ble = imu.LTx * 100;
  // RTx_ble = imu.RTx * 100;

  LKx = LSx - LTx;  // Left Knee
  RKx = RSx - RTx;  // Right Knee
  RLKx = RKx - LKx; // right-left knee angle difference
  LKx_filtered_last = LKx_filtered;
  LKx_filtered = ((0.95 * LKx_filtered_last) + (0.05 * LKx));
  RKx_filtered_last = RKx_filtered;
  RKx_filtered = ((0.95 * RKx_filtered_last) + (0.05 * RKx));
  RLKx_filtered = RKx_filtered - LKx_filtered;
  y_filtered = (sin(RKx_filtered * PI / 180) - sin(LKx_filtered * PI / 180));
  y_delay[doi] = y_filtered;
  RLKx_delay[doi] = RLKx_filtered;
  currentpoint = doi;
  delayindex = doi - delaypoint;
  if (delayindex < 0)
  {
    delayindex = delayindex + 100;
  }
  doi++;
  doi = doi % 100;

  if ((RLKx_delay[delayindex] >= 0) && (RLKx_delay[delayindex] < 120))
  {
    DOTC[0] = -Gain_E * y_delay[delayindex]; // left knee torque
    DOTC[1] = Gain_F * y_delay[delayindex];  // right knee torque
  }
  else if ((RLKx_delay[delayindex] < 0) && (RLKx_delay[delayindex] > -120))
  {
    DOTC[1] = Gain_E * y_delay[delayindex];  // right knee torque
    DOTC[0] = -Gain_F * y_delay[delayindex]; // left knee torque
  }
  else
  {
    // DOTC[1]=0;
    // DOTC[0]=0;
  }
}

// squat torque assistant
void IMU::SquatTorqueCommand()
{
  SquatTorque = -0.5 * (52 * (-9.8) * (0.287 * (-1) * sin(TKx * PI / 180) + 0.441 * sin((-1) * (RTx + LTx) / 2 * PI / 180)) + 19.6 * (-9.8) * 0.245 * sin((-1) * (RTx + LTx) / 2 * PI / 180));
}
