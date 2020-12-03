/*
  cdc.c
  libavr32

  usb host routines for cdc driver
 */

// std
#include <stdint.h>

// asf
#include "uhc.h"
#include <string.h>
#include "print_funcs.h"

// libavr32
#include "conf_usb_host.h"
#include "events.h"
#include "cdc.h"
#include "monome.h"
#include "ftdi.h"

static u8 cdcConnected = false;

static u8 rxBuf[CDC_RX_BUF_SIZE];
static u8 rxBytes = 0;
static u8 rxBusy = false;
static event_t e;

static u8 cdc_connected(void);
static volatile u8 cdc_rx_busy(void);
static volatile u8 cdc_tx_busy(void);
static volatile u8 cdc_rx_bytes(void);
static u8* cdc_rx_buf(void);
static void cdc_read(void);
static void cdc_write(u8* data, u32 bytes);

static monome_device_t cdcDevice = {
  cdc_connected,
  cdc_rx_busy,
  cdc_tx_busy,
  cdc_rx_bytes,
  cdc_rx_buf,
  cdc_read,
  cdc_write,
};

// Open and configure the communication port.
u8 cdc_setup(void) {
  // USB Device CDC connected
  cdcConnected = true;

  // Open and configure USB CDC ports
  usb_cdc_line_coding_t cfg = {
     .dwDTERate   = CPU_TO_LE32(115200),
     .bCharFormat = CDC_STOP_BITS_1,
     .bParityType = CDC_PAR_NONE,
     .bDataBits   = 8,
  };
  uhi_cdc_open(0, &cfg);
  return check_mext_device(&cdcDevice);
}

// This callback is called when a USB device CDC is plugged or unplugged. 
// We'll post an event, the application can then open and configure the
// communication port with cdc_setup() if it wants to.
bool callback_cdc_change(uhc_device_t* dev, bool b_plug) {
  if (b_plug) {
     e.type = kEventCDCConnect;
     print_dbg("\r\n CDC OPEN");
  } else {
     cdcConnected = false;
     e.type = kEventCDCDisconnect;
  }
  return 0;
}

void callback_cdc_rx_notify(void) {
  rxBytes = uhi_cdc_get_nb_received(0);
  uhi_cdc_read_buf(0, rxBuf, rxBytes);
  rxBusy = false;
}

u8 cdc_connected(void) {
  return cdcConnected;
}

volatile u8 cdc_rx_busy(void) {
  return rxBusy;
}

volatile u8 cdc_tx_busy(void) {
  // FIXME Not sure what value makes sense here! (use uhi_cdc_is_tx_ready?)
  return 0;
}

volatile u8 cdc_rx_bytes(void) {
  return rxBytes;
}

static u8* cdc_rx_buf(void) {
  return rxBuf;
}

void cdc_read(void) {
  if (rxBusy == false) {
    rxBytes = 0;
    rxBusy = true;
    // This is a bit weird, but as far as I can tell calling uhi_cdc_read_buf
    // with size 0 will try to start a transfer from the endpoint, and
    // callback_cdc_rx_notify will then be called as soon as there is data.
    uhi_cdc_read_buf(0, rxBuf, 0);
  }
}

void cdc_write(u8* data, u32 bytes) {
  // This is documented as returning the number of data remaining, but always
  // returns 0.
  uhi_cdc_write_buf(0, data, bytes);
}
