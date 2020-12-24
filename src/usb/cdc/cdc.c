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

static u8 cdcConnected = false;

static u8 rxBuf[CDC_RX_BUF_SIZE];
static u16 rxBytes = 0;
static u8 wroteOnce = false;
static u8 rxBusy = false;
static u8 txBusy = false;
static event_t e;

static u8 cdc_connected(void);
static volatile u8 cdc_rx_busy(void);
static volatile u8 cdc_tx_busy(void);
static volatile u16 cdc_rx_bytes(void);
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
void callback_cdc_change(uhc_device_t* dev, bool b_plug) {
  if (b_plug) {
     e.type = kEventCDCConnect;
     print_dbg("\r\n CDC OPEN");
     rxBytes = 0;
     wroteOnce = false;
  } else {
     cdcConnected = false;
     e.type = kEventCDCDisconnect;
     print_dbg("\r\n CDC CLOSED");
  }
  // posting an event so the main loop can respond
  event_post(&e);
}

u8 cdc_connected(void) {
  return cdcConnected;
}

volatile u8 cdc_rx_busy(void) {
  return rxBusy;
}

volatile u8 cdc_tx_busy(void) {
  return txBusy;
}

volatile u16 cdc_rx_bytes(void) {
  return rxBytes;
}

static u8* cdc_rx_buf(void) {
  return rxBuf;
}

static void cdc_rx_done(usb_add_t add,
                        usb_ep_t ep,
                        uhd_trans_status_t stat,
                        iram_size_t nb) {
  rxBytes = nb;
  if (rxBytes) {
    (*monome_read_serial)();
  }
  rxBusy = false;
}

void cdc_read(void) {
  if (wroteOnce && !rxBusy) {
    rxBytes = 0;
    rxBusy = true;
    if (!uhi_cdc_in_run(0, rxBuf, CDC_RX_BUF_SIZE, &cdc_rx_done)) {
      rxBusy = false;
    }
  }
}

static void cdc_tx_done(usb_add_t add,
                        usb_ep_t ep,
                        uhd_trans_status_t stat,
                        iram_size_t nb) {
  txBusy = false;

  if (stat != UHD_TRANS_NOERROR) {
    print_dbg("\r\n CDC tx transfer callback error. status: 0x");
    print_dbg_hex((u32)stat);
  }
}

void cdc_write(u8* data, u32 bytes) {
  if (!txBusy) {
    txBusy = true;
    wroteOnce = true;
    if(!uhi_cdc_out_run(0, data, bytes, &cdc_tx_done)) {
      print_dbg("\r\n CDC tx transfer error");
      txBusy = false;
    }
  }
}
