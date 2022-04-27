// 2022 for Fred.Wang edit
// bluefredtaiwan@gmail.com
// Fake ADAS for 2015 Taiwan CRV-2.4S 4WD
// BUS 0 is on the None side (EON send 0x194,0x1FA,0x30c,0x33d)
// BUS 1 is on the CAR side
// BUS 2 is on the fake EPS side


uint8_t SUM_STATUS[8]    = {0U,0U,0U,0U,0U,0U,0U,0U};
uint8_t EPS_STATUS[6]    = {0xF2,0xB0,0xFF,0xD8,0x00,0x00};
uint8_t POWER_DATA[8]    = {0U,0U,0U,0U,0U,0U,0U,0U};
uint8_t ACC_HUD_DATA[8]  = {0x00U,0x00U,0x00U,0xFFU,0x05U,0xE0U,0xC1U,0U};
uint8_t CRUISE_DATA[8]   = {0x00U,0x00U,0x00U,0xFFU,0x05U,0xE0U,0xC1U,0U};
uint8_t BRAKE_DATA[8]    = {0x00U,0x00U,0x00U,0xFFU,0x05U,0xE0U,0xC1U,0U};




uint8_t idx_37c     = 0U;
uint8_t idx_33d     = 0U;
uint8_t idx_30c     = 0U;
uint8_t idx_1fa     = 0U;
uint8_t idx_1b0     = 0U;
uint8_t idx_194     = 0U;
uint8_t idx_191     = 0U;
uint8_t idx_18f     = 0U;
uint8_t idx_17c     = 0U;
uint8_t idx_324     = 0U;
uint8_t idx_1fa0    = 0U;


uint8_t X_RELAY     = 0U;
uint8_t X_PEDAL_GAS = 0U;
uint8_t X_SPEED_H   = 0U;
uint8_t X_SPEED_L   = 0U;
uint8_t C_SPEED_PCM = 0U;

// Regsister 
int controls_acc_state = 0;

static uint8_t honda_compute_checksum(CAN_FIFOMailBox_TypeDef *to_fwd, int len, unsigned int addr) {
  uint8_t checksum = 0U;
  uint8_t byte = 0U;

  SUM_STATUS[0] = (to_fwd->RDLR & 0xFF);          // byte0
  SUM_STATUS[1] = (to_fwd->RDLR >> 8) & 0xFF;     // byte1
  SUM_STATUS[2] = (to_fwd->RDLR >> 16) & 0xFF;    // byte2
  SUM_STATUS[3] = (to_fwd->RDLR >> 24) & 0xFF;    // byte3
  SUM_STATUS[4] = (to_fwd->RDHR & 0xFF);          // byte4  
  SUM_STATUS[5] = (to_fwd->RDHR >> 8) & 0xFF;     // byte5
  SUM_STATUS[6] = (to_fwd->RDHR >> 16) & 0xFF;    // byte6
  SUM_STATUS[7] = (to_fwd->RDHR >> 24) & 0xFF;    // byte7


  while (addr > 0U) {
    checksum += (addr & 0xFU); 
    addr >>= 4;
  }
  for (int j = 0; j < len; j++) {
    byte = SUM_STATUS[j];
    checksum += (byte & 0xFU) + (byte >> 4U);
    if (j == (len - 1)) {
      checksum -= (byte & 0xFU);  // remove checksum in message
    }
  }
  return (8U - checksum) & 0xFU;
}

static int gm_ascm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  uint32_t addr = to_fwd->RIR>>21;
  uint8_t sum = 0U;
  

// CAN 1
  if (bus_num == 1) {

//  bus_fwd = 2;
  if (addr == 0x1A6) {
    int buttons = (to_fwd->RDLR & 0xE0) >> 5;
    if (buttons == 4 || buttons == 3) {
      controls_acc_state = 1;
    } else if (buttons == 2) {
      controls_acc_state = 0;
    }
    bus_fwd = 0;
  }

  else if (addr == 0x158) {
    X_SPEED_H = (to_fwd->RDLR & 0xFF);      // byte 0 XMISSION_SPEED bit7
    X_SPEED_L = (to_fwd->RDLR >> 8) & 0xFF; // byte 1 XMISSION_SPEED bit15
    bus_fwd = 0;
  }

// Force cruise speed = 100km (0x324 804 CRUISE: 8 PCM)
  else if (addr == 0x324) {
      C_SPEED_PCM = (to_fwd->RDHR & 0xFF); // CRUISE_SPEED_PCM
//    bus_fwd = 0;
      CRUISE_DATA[0] = (to_fwd->RDLR & 0xFF);
      CRUISE_DATA[1] = (to_fwd->RDLR >> 8) & 0xFF;
      CRUISE_DATA[2] = (to_fwd->RDLR >> 16) & 0xFF;
      CRUISE_DATA[3] = (to_fwd->RDLR >> 24) & 0xFF;
      
      if ((C_SPEED_PCM == 0xFF) || (C_SPEED_PCM < 0x2A))  {
        CRUISE_DATA[4] = 159;
      }
      else  {
        CRUISE_DATA[4] = (to_fwd->RDHR & 0xFF) + 15;
      }
//      CRUISE_DATA[4] = 0x64;// 0x64 = 100km // (to_fwd->RDHR & 0xFF);
      CRUISE_DATA[5] = (to_fwd->RDHR >> 8) & 0xFF;  
      CRUISE_DATA[6] = (to_fwd->RDHR >> 16) & 0xDF; 
      CRUISE_DATA[7] = 0U;
      
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x324 << 21 ; // addr
      (to_fwd->RDLR) = (CRUISE_DATA[0]) | (CRUISE_DATA[1] << 8) | (CRUISE_DATA[2] << 16) | (CRUISE_DATA[3] << 24);
      (to_fwd->RDHR) = (CRUISE_DATA[4]) | (CRUISE_DATA[5] << 8) | (CRUISE_DATA[6] << 16) | (CRUISE_DATA[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_324 << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x324);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);

      idx_324++;
      if (idx_324 >= 4U){
        idx_324 = 0U;
      }
     bus_fwd = 0;
  }



// ==== CAN0    
    // fake CAN POWERTRAIN_DATA 0x17C for acc status
// 0x17C, 20,00,00,00,00,01,0E,
  else if (addr == 0x17C){
      
      X_PEDAL_GAS   = (to_fwd->RDLR & 0xFF);
      POWER_DATA[0] = 0U; // (to_fwd->RDLR & 0xFF); // PEDAL_GAS = 0 byte0 
      POWER_DATA[1] = (to_fwd->RDLR >> 8) & 0xFF;
      POWER_DATA[2] = (to_fwd->RDLR >> 16) & 0xFF;
      POWER_DATA[3] = (to_fwd->RDLR >> 24) & 0xFF;

      POWER_DATA[4] = (to_fwd->RDHR & 0xFE); // BRAKE_SWITCH = 0 (byte4 bit0)
      if (controls_acc_state != 0){
         POWER_DATA[4] = (POWER_DATA[4] | 0x40); // ACC_STATUS = 1
       }
      else {
         POWER_DATA[4] = (POWER_DATA[4] & 0xBF); // ACC_STATUS = 0
       }
      
      POWER_DATA[5] = (to_fwd->RDHR >> 8) & 0xFF;  
      POWER_DATA[6] = (to_fwd->RDHR >> 16) & 0xDF; // BRAKE_PRESSED = 0 byte7 bit5
      POWER_DATA[7] = 0U;
      
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x17C << 21 ; // addr
      (to_fwd->RDLR) = (POWER_DATA[0]) | (POWER_DATA[1] << 8) | (POWER_DATA[2] << 16) | (POWER_DATA[3] << 24);
      (to_fwd->RDHR) = (POWER_DATA[4]) | (POWER_DATA[5] << 8) | (POWER_DATA[6] << 16) | (POWER_DATA[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_17c << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x17C);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);

      idx_17c++;
      if (idx_17c >= 4U){
        idx_17c = 0U;
      }
     bus_fwd = 0;
    }

    // fake CAN EPS 0x18F
    // 0x18F, 20,00,00,00,00,01,0E,
  else if (addr == 0x18F){
      
      EPS_STATUS[0] = (to_fwd->RDLR & 0xFF);
      EPS_STATUS[1] = (to_fwd->RDLR >> 8) & 0xFF;
      EPS_STATUS[2] = (to_fwd->RDLR >> 16) & 0xFF;
      EPS_STATUS[3] = (to_fwd->RDLR >> 24) & 0xFF;
      // byte4 for test
      // EPS_STATUS[4] = (to_fwd->RDHR & 0xFF);
      EPS_STATUS[4] = (to_fwd->RDHR & 0xF0); // for test
      EPS_STATUS[5] = 0U;
      (to_fwd->RDTR) = (6 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x18F << 21 ; // addr
      (to_fwd->RDLR) = (EPS_STATUS[0]) | (EPS_STATUS[1] << 8) | (EPS_STATUS[2] << 16) | (EPS_STATUS[3] << 24);
      (to_fwd->RDHR) = (EPS_STATUS[4]) | (EPS_STATUS[5] << 8);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_18f << 12);
      sum = honda_compute_checksum(to_fwd, 6, 0x18F);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 8);

      idx_18f++;
      if (idx_18f >= 4U){
        idx_18f = 0U;
      }
     bus_fwd = 0;
    }

    // fake CAN STANDSTILL 0x1B0
    // 0x1B0, 20,00,00,00,00,01,0E,
  else if (addr == 0x1B0){
      (to_fwd->RDTR) = (7 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x1B0 << 21 ;
      (to_fwd->RDHR) = 0x000000 + (idx_1b0 << 20);   //byte7 byte6 byte5 byte4
      (to_fwd->RDLR) = 0x00000000 ;                  //byte3 byte2 byte1 byte0 // 01234567 Low   4 Byte      high bit7
      sum = honda_compute_checksum(to_fwd, 7, 0x1B0);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 16);

      idx_1b0++;
      if (idx_1b0 >= 4U){
        idx_1b0 = 0U;
      }
      bus_fwd = 0;
    }

    // fake CAN GEARBOX 0x191
    // 0x191,08,01 00 00 A9 A7 01 7F 2F ,,
    //          04 00 00 a9 a9 23 01 0d
  else if (addr == 0x188){
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x191<<21 ;
      (to_fwd->RDHR) = 0x000123A9 + (idx_191 << 28);
      (to_fwd->RDLR) = 0xA9000008 ; // high
      sum = honda_compute_checksum(to_fwd, 8, 0x191);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);

      idx_191++;
      if (idx_191 >= 4U){
        idx_191 = 0U;
      }
      bus_fwd = 0;
    }

    // 0x37C 892 CRUISE_PARAMS
  else if (addr == 0x1AA){
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x37C<<21 ;
      (to_fwd->RDHR) = 0x000123A9 + (idx_37c << 28);
      (to_fwd->RDLR) = 0x00000000 ; // high
      sum = honda_compute_checksum(to_fwd, 8, 0x37C);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);

      idx_37c++;
      if (idx_37c >= 4U){
        idx_37c = 0U;
      }
      bus_fwd = 0;
    }

    // ==== CAN2    
    // fake CAN STEERING_CONTROL 0x194
  else if (addr == 0x13C){
      (to_fwd->RDTR) = (4 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x194<<21 ;
      (to_fwd->RDLR) = 0x00000000 + (idx_194 << 28); // high
      sum = honda_compute_checksum(to_fwd, 4, 0x194);
      to_fwd->RDLR = to_fwd->RDLR + (sum << 24);

      idx_194++;
      if (idx_194 >= 4U){
        idx_194 = 0U;
      }
      bus_fwd = 2;
    }

    // fake CAN BRAKE_COMMAND 0x1FA ,0x1FA is brake control
  else if (addr == 0x1DC){
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x1FA<<21 ;
      (to_fwd->RDHR) = 0x00000000 + (idx_1fa << 28); // low 
//      (to_fwd->RDLR) = 0x80120000 ; // CRUISE_CANCEL_CMD = 1 = bit17
//      (to_fwd->RDLR) = 0x80110000 ; // COMPUTER_BRAKE_REQUEST = 1 = bit18
      (to_fwd->RDLR) = 0x80100000 ; // Perset
      sum = honda_compute_checksum(to_fwd, 8, 0x1FA);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);

      idx_1fa++;
      if (idx_1fa >= 4U){
        idx_1fa = 0U;
      }
      bus_fwd = 2;
    }
    
    // fake CAN ACC_HUD 0x30C ,0x30C is acc hud
  else if (addr == 0x1ED){
      ACC_HUD_DATA[0] = X_SPEED_H;
      ACC_HUD_DATA[1] = X_SPEED_L;
      ACC_HUD_DATA[3] = C_SPEED_PCM;
      ACC_HUD_DATA[7] = 0U;
      
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x30C << 21 ; // addr
      (to_fwd->RDLR) = (ACC_HUD_DATA[0]) | (ACC_HUD_DATA[1] << 8) | (ACC_HUD_DATA[2] << 16) | (ACC_HUD_DATA[3] << 24);
      (to_fwd->RDHR) = (ACC_HUD_DATA[4]) | (ACC_HUD_DATA[5] << 8) | (ACC_HUD_DATA[6] << 16) | (ACC_HUD_DATA[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_30c << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x30C);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);
     
      idx_30c++;
      if (idx_30c >= 4U){
        idx_30c = 0U;
      }
      bus_fwd = 2;
    }

    // fake CAN LKAS_HUD 0x33D ,0x33D is lkas hud
  else if (addr == 0x400){
      (to_fwd->RDTR) = (5 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x33D << 21 ;
      (to_fwd->RDHR) = 0x00 + (idx_33d); // low 
      (to_fwd->RDLR) = 0x48000441 ; // high
      sum = honda_compute_checksum(to_fwd, 5, 0x33D);
      to_fwd->RDHR = to_fwd->RDHR + (sum);
      
      idx_33d++;
      if (idx_33d >= 4U){
        idx_33d = 0U;
      }
      bus_fwd = 2;
    }
    // Mask CAN ID
  else if ((addr == 0x156)||(addr == 0x305)||(addr == 0x309)||(addr == 0x37B)||
           (addr == 0x405)||(addr == 0x1A4)||(addr == 0x294)||(addr == 0x255)||
           (addr == 0x1D0)||(addr == 0x1E7)||(addr == 0x1EA)) {
      bus_fwd = 0;
    }
  else {
    bus_fwd = -1;
   }
  }
// end CAN1

// CAN 2
  if (bus_num == 2) {
    bus_fwd = -1;
  }

// CAN 0
  if (bus_num == 0){
    // EON or Send CAN BRAKE_COMMAND 0x1FA ,0x1FA is brake control
    if (addr == 0x1FA) {
      
      BRAKE_DATA[0] = (to_fwd->RDLR & 0xFF);
      X_RELAY       = BRAKE_DATA[0];
      BRAKE_DATA[1] = (to_fwd->RDLR >> 8)  & 0xFF;
      BRAKE_DATA[2] = (to_fwd->RDLR >> 16) & 0xFF;
      BRAKE_DATA[3] = (to_fwd->RDLR >> 24) & 0xFF;

      BRAKE_DATA[4] = (to_fwd->RDHR & 0xFF);
      BRAKE_DATA[5] = (to_fwd->RDHR >> 8)   & 0xFF;
      BRAKE_DATA[6] = (to_fwd->RDHR >> 16)  & 0xFF;
      BRAKE_DATA[7] = 0U;
      // USER PEDAL_GAS > EON BRAKE_COMMAND
      if (X_PEDAL_GAS) {
        BRAKE_DATA[0] = 0;                    // COMPUTER_BRAKE = 0      bit7~(10)
        BRAKE_DATA[1] = BRAKE_DATA[1] & 0x3E; // BRAKE_PUMP_REQUEST = 0  bit8
        BRAKE_DATA[2] = BRAKE_DATA[2] & 0xFD; // CRUISE_CANCEL_CMD = 0   bit17
       }
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x1FA << 21 ; // addr
      (to_fwd->RDLR) = (BRAKE_DATA[0]) | (BRAKE_DATA[1] << 8) | (BRAKE_DATA[2] << 16) | (BRAKE_DATA[3] << 24);
      (to_fwd->RDHR) = (BRAKE_DATA[4]) | (BRAKE_DATA[5] << 8) | (BRAKE_DATA[6] << 16) | (BRAKE_DATA[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_1fa0 << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x1FA);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);
     
      idx_1fa0++;
      if (idx_1fa0 >= 4U){
        idx_1fa0 = 0U;
      }
      bus_fwd = 1;
    }
    else if ((addr == 0x194)||(addr == 0x30C)||(addr == 0x33D)){
//    else if ((addr == 0x194)||(addr == 0x1FA)||(addr == 0x30C)||(addr == 0x33D)){
      
      bus_fwd = 1;
    }
    else {
      bus_fwd = -1;
    }
  }
  return bus_fwd ;
}

const safety_hooks gm_ascm_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = gm_ascm_fwd_hook,
  .relay = nooutput_relay_hook,
};

