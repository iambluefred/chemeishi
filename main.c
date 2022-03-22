// ********************* Includes *********************
// V0.8.8
#include "../config.h"

#include "early_init.h"
#include "crc.h"

#define CAN CAN1

#ifdef PEDAL_USB
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeefU
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early_initialization();
}

// ********************* serial debugging *********************

#ifdef PEDAL_USB

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete(void) {}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xc1: get hardware type
    case 0xc1:
      resp[0] = hw_type;
      resp_len = 1;
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** BSM can checksum *****************************
int bsm_can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}

// ***************************** can port *****************************

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

#define MAX_TIMEOUT 10U
uint32_t timeout = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
uint8_t state = FAULT_STARTUP;
// const uint8_t crc_poly = 0xD5U;  // standard crc8

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

// toyota bsm
#define CAN_BSM_OUTPUT 0x3f6

int RightBSM   = 0;
int LeftBSM    = 0;
int led_value  = 0;
uint8_t BSMdat[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t BSMa   = 0;
uint8_t BSMb   = 0;


void TIM3_IRQ_Handler(void) {

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    BSMdat[0] = BSMa ;
    BSMdat[1] = BSMb ;
    BSMdat[7] = bsm_can_cksum(BSMdat, 7, 0x3F6);
    CAN->sTxMailBox[0].TDLR = BSMdat[0] | (BSMdat[1]<<8) | (BSMdat[2]<<16) | (BSMdat[3]<<24);
    CAN->sTxMailBox[0].TDHR = BSMdat[4] | (BSMdat[5]<<8) | (BSMdat[6]<<16) | (BSMdat[7]<<24);
    CAN->sTxMailBox[0].TDTR = 8;  // len of packet is 8
    CAN->sTxMailBox[0].TIR = (CAN_BSM_OUTPUT << 21) | 1U;

  }

  // blink the LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}

// ***************************** main code *****************************


void BSMDetect(void) {
  // read/write
  RightBSM = adc_get(ADCCHAN_ACCEL0); // Right BSM 
  LeftBSM  = adc_get(ADCCHAN_ACCEL1); // Left BSM
  
  // Right BSM
  if (RightBSM < 300)  {
    BSMa = BSMa | 0x82;
    BSMb = BSMb | 0x84;
   }
  else {
    BSMa = BSMa & 0xFD;
    BSMb = BSMb & 0xFB;
  }
    
  // Left BSM
  if (LeftBSM < 300) {
    BSMa = BSMa | 0x81;
    BSMb = BSMb | 0x81;
   }
  else {
    BSMa = BSMa & 0xFE;
    BSMb = BSMb & 0xFE;
  }

  if ((RightBSM > 300) && (LeftBSM > 300)) {
    BSMa = 0x00;
    BSMb = 0x00;
   }
  // feed the watchdog
  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  
  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_external_debug_serial();
  detect_board_type();

  // init board
  current_board->init();

#ifdef PEDAL_USB
  // enable USB
  usb_init();
#endif

  // pedal stuff
  dac_init();
  adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 75); // 100mS
//  timer_init(TIM3, 15); // 20mS
  NVIC_EnableIRQ(TIM3_IRQn);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  puts("**** Toyota BSM CANBus ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {

    BSMDetect();
  }

  return 0;
}
