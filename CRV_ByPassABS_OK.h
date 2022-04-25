// 2022 for Fred.Wang edit
// bluefredtaiwan@gmail.com
// BUS 0 no use
// BUS 1 is on the ABS module side
// BUS 2 is on the Dashboard module side
// Honda 2015 CRV S 4WD
// ABS CAN1 <---> CAN2 Dashboard (reciver fake 0x1b0 ,0x1a4)

uint8_t SUM_STATUS[8]  = {0U,0U,0U,0U,0U,0U,0U,0U};

uint8_t STAND_STILL[7]  = {0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t VSA_STATUS[8]   = {0U,0U,0U,0U,0U,0U,0U,0U};

// Modify CAN data to Dashboard
uint8_t idx_1b0 = 0U;
uint8_t idx_1a4 = 0U;



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

// CAN 2 >> 1
  if (bus_num == 2) {
      can_bus_num = 1;
    }
// CAN1 >> 2
  if (bus_num == 1)  {
    
    can_bus_num = 2;

    // Check 0x1B0 432 STANDSTILL: 7 VSA
    if (addr == 0x1B0){
      
      STAND_STILL[0] = (to_fwd->RDLR & 0xFF);
      STAND_STILL[1] = (to_fwd->RDLR >> 8) & 0xDF; // MASK BIT5
      STAND_STILL[2] = (to_fwd->RDLR >> 16) & 0xF0; // MASK BIT3/2/1/0
      STAND_STILL[3] = (to_fwd->RDLR >> 24) & 0xFF;

      STAND_STILL[4] = (to_fwd->RDHR & 0xFF);
      STAND_STILL[5] = (to_fwd->RDHR >> 8) & 0xFE; // MASK BIT0
      STAND_STILL[6] = 0U;

      (to_fwd->RDTR) = (7 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x1B0 << 21 ; // addr
      (to_fwd->RDLR) = (STAND_STILL[0]) | (STAND_STILL[1] << 8) | (STAND_STILL[2] << 16) | (STAND_STILL[3] << 24);
      (to_fwd->RDHR) = (STAND_STILL[4]) | (STAND_STILL[5] << 8) | (STAND_STILL[6] << 16) ;

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_1b0 << 20);
      sum = honda_compute_checksum(to_fwd, 7, 0x1B0);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 16);

      idx_1b0++;
      if (idx_1b0 >= 4U){
        idx_1b0 = 0U;
      }
    }
    // Check 0x1A4 420 VSA_STATUS: 8 VSA
    if (addr == 0x1A4){
      
      VSA_STATUS[0] = (to_fwd->RDLR & 0xFF);
      VSA_STATUS[1] = (to_fwd->RDLR >> 8) & 0xFF; 
      VSA_STATUS[2] = (to_fwd->RDLR >> 16) & 0xFF; // 
      VSA_STATUS[3] = (to_fwd->RDLR >> 24) & 0xDF; // MASK BIT5

      VSA_STATUS[4] = (to_fwd->RDHR & 0xFF);
      VSA_STATUS[5] = (to_fwd->RDHR >> 8) & 0xFF; // 
      VSA_STATUS[6] = (to_fwd->RDHR >> 16) & 0xFE; // MASK BIT0
      VSA_STATUS[7] = 0U;

      (to_fwd->RDTR) = (8 & 0Xf) ; // len
      (to_fwd->RIR ) = 0x1a4 << 21 ; // addr
      (to_fwd->RDLR) = (VSA_STATUS[0]) | (VSA_STATUS[1] << 8) | (VSA_STATUS[2] << 16) | (VSA_STATUS[3] << 24);
      (to_fwd->RDHR) = (VSA_STATUS[4]) | (VSA_STATUS[5] << 8) | (VSA_STATUS[6] << 16) | (VSA_STATUS[7] << 24);

      (to_fwd->RDHR) = (to_fwd->RDHR) + (idx_1a4 << 28);
      sum = honda_compute_checksum(to_fwd, 8, 0x1A4);
      to_fwd->RDHR = to_fwd->RDHR + (sum << 24);

      idx_1a4++;
      if (idx_1a4 >= 4U){
        idx_1a4 = 0U;
      }
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




