//#define DEBUG
//#define CAN_LOOPBACK_MODE
//#define USE_INTERNAL_OSC

#include "../config.h"

#include "drivers/drivers.h"
#include "drivers/llgpio.h"
#include "gpio.h"

#define CUSTOM_CAN_INTERRUPTS

#include "libc.h"
#include "safety.h"
#include "drivers/adc.h"
#include "drivers/uart.h"
#include "drivers/dac.h"
#include "drivers/can.h"
#include "drivers/timer.h"

#define CAN CAN1

//#define PEDAL_USB

#ifdef PEDAL_USB
  #include "drivers/usb.h"
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

void __initialize_hardware_early() {
  early();
}

// ********************* serial debugging *********************

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv)) {
    putc(ring, rcv);
  }
}

#ifdef PEDAL_USB

int usb_cb_ep1_in(uint8_t *usbdata, int len, int hardwired) { return 0; }
void usb_cb_ep2_out(uint8_t *usbdata, int len, int hardwired) {}
void usb_cb_ep3_out(uint8_t *usbdata, int len, int hardwired) {}
void usb_cb_enumeration_complete() {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, int hardwired) {
  int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) break;
      if (ur == &esp_ring) uart_dma_drain();
      // read
      while ((resp_len < min(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
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

uint8_t pedal_checksum(uint8_t *dat, int len) {
  uint8_t crc = 0xFF;
  uint8_t poly = 0xD5; // standard crc8
  int i, j;
  for (i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0) {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_GAS_INPUT  0x3F5
#define CAN_BSM_OUTPUT 0x3F6
#define CAN_BSM_SIZE 8
#define COUNTER_CYCLE 0xF

void CAN1_TX_IRQHandler() {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// two independent values
uint16_t gas_set_0 = 0;
uint16_t gas_set_1 = 0;

#define MAX_TIMEOUT 10
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0
#define FAULT_BAD_CHECKSUM 1
#define FAULT_SEND 2
#define FAULT_SCE 3
#define FAULT_STARTUP 4
#define FAULT_TIMEOUT 5
#define FAULT_INVALID 6
uint8_t state = FAULT_STARTUP;

void CAN1_RX0_IRQHandler() {
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    uint32_t address = CAN->sFIFOMailBox[0].RIR>>21;
    if (address == CAN_GAS_INPUT) {
      // softloader entry
      if (CAN->sFIFOMailBox[0].RDLR == 0xdeadface) {
        if (CAN->sFIFOMailBox[0].RDHR == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (CAN->sFIFOMailBox[0].RDHR == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        }
      }

      // normal packet
      uint8_t dat[8];
      uint8_t *rdlr = (uint8_t *)&CAN->sFIFOMailBox[0].RDLR;
      uint8_t *rdhr = (uint8_t *)&CAN->sFIFOMailBox[0].RDHR;
      for (int i=0; i<4; i++) {
        dat[i] = rdlr[i];
        dat[i+4] = rdhr[i];
      }
      uint16_t value_0 = (dat[0] << 8) | dat[1];
      uint16_t value_1 = (dat[2] << 8) | dat[3];
      uint8_t enable = (dat[4] >> 7) & 1;
      uint8_t index = dat[4] & COUNTER_CYCLE;
      if (pedal_checksum(dat, CAN_BSM_SIZE - 1) == dat[7]) {
        if (((current_index + 1) & COUNTER_CYCLE) == index) {
          #ifdef DEBUG
            puts("setting gas ");
            puth(value);
            puts("\n");
          #endif
          if (enable) {
            gas_set_0 = value_0;
            gas_set_1 = value_1;
          } else {
            // clear the fault state if values are 0
            if (value_0 == 0 && value_1 == 0) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID;
            }
            gas_set_0 = gas_set_1 = 0;
          }
          // clear the timeout
          timeout = 0;
        }
        current_index = index;
      } else {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQHandler() {
  state = FAULT_SCE;
  can_sce(CAN);
}

//int pdl0 = 0, pdl1 = 0;
int RightBSM   = 0;
int LeftBSM    = 0;
int led_value  = 0;
uint8_t BSMdat[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t BSMa   = 0;
uint8_t BSMb   = 0;

void TIM3_IRQHandler() {

  // check timer for sending the Toyota BSM CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {

    BSMdat[0] = BSMa ;
    BSMdat[1] = BSMb ;
    BSMdat[7] = bsm_can_cksum(BSMdat, 7, 0x3F6);
    CAN->sTxMailBox[0].TDLR = BSMdat[0] | (BSMdat[1]<<8) | (BSMdat[2]<<16) | (BSMdat[3]<<24);
    CAN->sTxMailBox[0].TDHR = BSMdat[4] | (BSMdat[5]<<8) | (BSMdat[6]<<16) | (BSMdat[7]<<24);
    CAN->sTxMailBox[0].TDTR = 8;  // len of packet is 8
    CAN->sTxMailBox[0].TIR = (CAN_BSM_OUTPUT << 21) | 1;
  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1;
  }
}

// ***************************** main code *****************************

void BSMDetect() {
  // read/write
  RightBSM = adc_get(ADCCHAN_ACCEL0); // Right BSM 
  LeftBSM  = adc_get(ADCCHAN_ACCEL1); // Left BSM
  
  // Right BSM
  if (RightBSM < 100) {
    BSMa = 0x82;
    BSMb = 0x84;
   }
  else {
    BSMa = BSMa & 0xFD;
    BSMb = BSMb & 0xFB;
  }
    
  // Left BSM
  if (LeftBSM < 100) {
    BSMa = BSMa | 0x81;
    BSMb = BSMb | 0x81;
   }
  else {
    BSMa = BSMa & 0xFE;
    BSMb = BSMb & 0xFE;
  }

  if ((RightBSM > 100) && (LeftBSM > 100)) {
    BSMa = 0x00;
    BSMb = 0x00;
   }
  // feed the watchdog
  IWDG->KR = 0xAAAA;
}

int main() {
  __disable_irq();

  // init devices
  clock_init();
  periph_init();
  gpio_init();

#ifdef PEDAL_USB
  // enable USB
  usb_init();
#endif

  // pedal stuff
  dac_init();
  adc_init();

  // init can
  can_silent = ALL_CAN_LIVE;
  can_init(0);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  // setup watchdog
  IWDG->KR = 0x5555;
  IWDG->PR = 0;          // divider /4
  // 0 = 0.125 ms, let's have a 50ms watchdog
  IWDG->RLR = 400 - 1;
  IWDG->KR = 0xCCCC;

  puts("**** INTERRUPTS ON ****\n");
  __enable_irq();


  // main BSM detect loop
  while (1) {
    BSMDetect();
  }

  return 0;
}
