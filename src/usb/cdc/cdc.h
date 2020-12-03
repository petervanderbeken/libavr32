#ifndef _USB_CDC_H_
#define _USB_CDC_H_

#include "uhc.h"
#include "types.h"

#define CDC_RX_BUF_SIZE 256

extern bool callback_cdc_change(uhc_device_t* dev, bool b_plug);
extern void callback_cdc_rx_notify(void);

extern u8 cdc_setup(void);

#endif
