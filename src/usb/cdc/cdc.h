#ifndef _USB_CDC_H_
#define _USB_CDC_H_

#include "uhc.h"
#include "types.h"

#define CDC_RX_BUF_SIZE (5 * 64)

extern void callback_cdc_change(uhc_device_t* dev, bool b_plug);

extern u8 cdc_setup(void);

#endif
