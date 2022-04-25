// 2022 for Fred.Wang edit
// bluefredtaiwan@gmail.com
// BUS 0 no use
// BUS 1 is on the wheel module side
// BUS 2 is on the EPS module side
// 2015 Taiwan CRV 2.4S
// wheel CAN1 <---> CAN2 EPS (fake 38km for 0x255, 0x1d0, 0x158)
// by pass eps for 39980-t1w-a0

uint8_t SUM_STATUS[8]        = {0U,0U,0U,0U,0U,0U,0U,0U};
uint8_t ENGINE_STATUS[8]     = {0U,0U,0U,0U,0U,0U,0U,0U}; // 0x158

uint8_t WHEEL_SPEED[8]       = {0x00, 0x00, 0x00, 0x00, 0x55, 0x55, 0x25, 0x00}; // 0x1D0
uint8_t ROU_WHEEL_SPEED[8]   = {0x00, 0x00, 0x00, 0x00, 0x55, 0x55, 0x25, 0x00}; // 0x255

// Make meter
uint8_t idx_255 = 0U;
uint8_t idx_1d0 = 0U;
uint8_t idx_158 = 0U;

// Flage
uint8_t e_go_speed       = 0;
uint8_t by_pass_EPS      = 0;
uint8_t openpilotEnable  = 0;


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

  int can_bus_num = -1;
  uint32_t addr = to_fwd->RIR>>21;
  uint8_t sum = 0U;
  
  // 0x194 control EPS unit
  if (addr == 0x194) {
    int lkas_active = (to_fwd->RDLR >> 16) & 0x80;

    if (lkas_active == 0x80) {
      openpilotEnable = 1;
     } 
    else {
      openpilotEnable = 0;
     }
  }
  
// CAN 2 >> 1
  if (bus_num == 2) {
      can_bus_num = 1;
    }
// CAN1 >> 2
  if (bus_num == 1)  {

// ==== CAN1 TO CAN2
    can_bus_num = 2;
    // Check & Modify data 0x158 344 ENGINE_DATA: 8 PCM
    // XMISSION_SPEED  = 0x0ED8 = 38km
    // XMISSION_SPEED2 = 0x0ED8 = 38km
    // ENGINE_RPM      = 0x0334 = 820rpm
    if ((addr == 0x158)&&(openpilotEnable)) {
      
      e_go_speed = (to_fwd->RDLR & 0xFF); // EPS_speed = 10km < e_go_speed < 35km , by_pass_EPS = 1
      if ((e_go_speed < 0x0E) && (e_go_speed > 0x04)) { 
//      if ((e_go_speed < 0x12) && (e_go_speed > 0x3)) { 
        by_pass_EPS = 1; // yes by pass eps unit
        ENGINE_STATUS[0] = 0x0E; // XMISSION_SPEED high;
        ENGINE_STATUS[1] = 0xD8; // XMISSION_SPEED low;
        ENGINE_STATUS[2] = 0x03; // ENGINE_RPM high;
        ENGINE_STATUS[3] = 0x34; // ENGINE_RPM low;
  
        ENGINE_STATUS[4] = 0x0E; // XMISSION_SPEED2 high;
        ENGINE_STATUS[5] = 0xD8; // XMISSION_SPEED2low;
        ENGINE_STATUS[6] = (to_fwd->RDHR >> 16) & 0xFF; 
        idx_158          = (to_fwd->RDHR >> 28) & 0x0F; // count
        ENGINE_STATUS[7] = 0U;

        (to_fwd->RDTR) = (8 & 0Xf) ; // len
        (to_fwd->RIR ) = 0x158 << 21 ; // addr
        (to_fwd->RDLR) = (ENGINE_STATUS[0]) | (ENGINE_STATUS[1] << 8) | (ENGINE_STATUS[2] << 16) | (ENGINE_STATUS[3] << 24);
        (to_fwd->RDHR) = (ENGINE_STATUS[4]) | (ENGINE_STATUS[5] << 8) | (ENGINE_STATUS[6] << 16) | (ENGINE_STATUS[7] << 24);
 
        (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_158 << 28);
        sum = honda_compute_checksum(to_fwd, 8, 0x158);
        to_fwd->RDHR = to_fwd->RDHR + (sum << 24);
      }
      else {
        by_pass_EPS = 0; // no by pass eps unit
       }
    }

    // Modify data 0x255 597 ROUGH_WHEEL_SPEED: 8 VSA
    // 0x26 = 38km
    if ((addr == 0x255)&&(by_pass_EPS)&&(openpilotEnable)) {

      ROU_WHEEL_SPEED[0] = 0x26;
      ROU_WHEEL_SPEED[1] = 0x26;
      ROU_WHEEL_SPEED[2] = 0x26;
      ROU_WHEEL_SPEED[3] = 0x26;

      ROU_WHEEL_SPEED[4] = (to_fwd->RDHR & 0xFF);
      ROU_WHEEL_SPEED[5] = (to_fwd->RDHR >> 8) & 0xFF;
      ROU_WHEEL_SPEED[6] = (to_fwd->RDHR >> 16) & 0xFF;
      idx_255            = (to_fwd->RDHR >> 28) & 0x0F;
      ROU_WHEEL_SPEED[7] = 0U;
      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x255 << 21 ; // addr
      (to_fwd->RDLR) = (ROU_WHEEL_SPEED[0]) | (ROU_WHEEL_SPEED[1] << 8) | (ROU_WHEEL_SPEED[2] << 16) | (ROU_WHEEL_SPEED[3] << 24);
      (to_fwd->RDHR) = (ROU_WHEEL_SPEED[4]) | (ROU_WHEEL_SPEED[5] << 8) | (ROU_WHEEL_SPEED[6] << 16) | (ROU_WHEEL_SPEED[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_255 << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x255);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);
    }

    // Modify data 0x1D0 464 WHEEL_SPEEDS: 8 VSA
    // 0x0ED8 = 36km
    if ((addr == 0x1D0)&&(by_pass_EPS)&&(openpilotEnable)) {
      WHEEL_SPEED[0] = 0x1D;
      WHEEL_SPEED[1] = 0xB0;
      WHEEL_SPEED[2] = 0x3B;
      WHEEL_SPEED[3] = 0x60;

      WHEEL_SPEED[4] = 0x76;
      WHEEL_SPEED[5] = 0xC0;
      WHEEL_SPEED[6] = 0xED; 
      idx_1d0        = 0x00; // no idx
      WHEEL_SPEED[7] = 0U;

      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x1D0 << 21 ; // addr
      (to_fwd->RDLR) = (WHEEL_SPEED[0]) | (WHEEL_SPEED[1] << 8) | (WHEEL_SPEED[2] << 16) | (WHEEL_SPEED[3] << 24);
      (to_fwd->RDHR) = (WHEEL_SPEED[4]) | (WHEEL_SPEED[5] << 8) | (WHEEL_SPEED[6] << 16) | (WHEEL_SPEED[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_1d0 << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x1D0);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);
    }
  }
   
// CAN 0
  if (bus_num == 0) {
     can_bus_num = -1;
  }
  return can_bus_num;
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
