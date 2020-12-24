/**
 * \file
 *
 * \brief USB host driver for Communication Device Class interface.
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * This is a simplified CDC driver, mimicking the FTDI driver. It uses some code
 * from the CDC host code in the ASF, mostly for setting up the UDC device. The
 * rest of the code resembles the FTDI code, to avoid rewriting a lot of the
 * monome device code, and at this point it only supports one CDC port.
 */

#ifndef _UHI_CDC_H_
#define _UHI_CDC_H_

#include "conf_usb_host.h"
#include "usb_protocol_cdc.h"
#include "uhc.h"
#include "uhi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UHI_CDC { \
	.install = uhi_cdc_install, \
	.enable = uhi_cdc_enable, \
	.uninstall = uhi_cdc_uninstall, \
	.sof_notify = NULL, \
}

uhc_enum_status_t uhi_cdc_install(uhc_device_t* dev);
void uhi_cdc_enable(uhc_device_t* dev);
void uhi_cdc_uninstall(uhc_device_t* dev);

bool uhi_cdc_open(uint8_t port, usb_cdc_line_coding_t *configuration);
void uhi_cdc_close(uint8_t port);
bool uhi_cdc_in_run(uint8_t port, uint8_t* buf, iram_size_t buf_size,
              uhd_callback_trans_t callback);
bool uhi_cdc_out_run(uint8_t port, uint8_t * buf, iram_size_t buf_size,
		      uhd_callback_trans_t callback);

/**
 * - \code
 *     #define UHI_CDC_CHANGE(dev, b_plug) my_callback_cdc_change(dev, b_plug)
 *     extern bool my_callback_cdc_change(uhc_device_t* dev, bool b_plug);
 *   \endcode
 *   \note This callback is called when a USB device CDC is plugged or unplugged.
 *   The communication port can be opened and configured here.
 */


#ifdef __cplusplus
}
#endif

#endif // _UHI_CDC_H_
