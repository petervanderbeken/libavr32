/**
 * \file
 *
 * \brief USB host Communication Device Class interface.
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

#include "conf_usb_host.h"
#include "usb_protocol.h"
#include "uhd.h"
#include "uhc.h"
#include "uhi_cdc.h"
#include <string.h>

#define UHI_CDC_TIMEOUT 20

//! Communication port information
typedef struct {
	//! Current port configuration (baudrate,...)
	usb_cdc_line_coding_t conf;
	//! USB CDC-COMM interface number
	uint8_t  iface_comm;
	//! USB CDC-DATA interface number
	uint8_t  iface_data;
	//! Interrupt IN endpoint from CDC-COMM interface
	usb_ep_t ep_comm_in;
	//! RX endpoint
	usb_ep_t ep_data_rx;
	//! TX endpoint
	usb_ep_t ep_data_tx;
} uhi_cdc_port_t;

//! USB CDC device information
typedef struct {
	//! Pointer on USB Device information
	uhc_device_t *dev;
	//! True, if a CDC Device has been enumerated
	bool b_enabled;
	//! Pointer on communication port(s) information
	uhi_cdc_port_t *port;
	//! Number of port available on this USB device
	uint8_t nb_port;
} uhi_cdc_dev_t;

//! Information about the enumerated USB CDC device
static uhi_cdc_dev_t uhi_cdc_dev = {
	.dev = NULL,
	.nb_port = 0,
};

/**
 * Internal function declarations.
 */
static void uhi_cdc_free_device(void);
static uhi_cdc_port_t* uhi_cdc_get_port(uint8_t port_num);
static bool uhi_cdc_set_conf(uint8_t port, usb_cdc_line_coding_t *configuration);
static bool uhi_cdc_set_ctrl_line(uint8_t port, le16_t wValue);

/**
 * Functions for use by the UHC module.
 */

uhc_enum_status_t uhi_cdc_install(uhc_device_t* dev)
{
	bool b_iface_comm, b_iface_data;
	uint16_t conf_desc_lgt;
	uint8_t port_num, i;
	usb_iface_desc_t *ptr_iface;
	uhi_cdc_port_t *ptr_port = NULL;

	if (uhi_cdc_dev.dev != NULL) {
		return UHC_ENUM_SOFTWARE_LIMIT; // Device already allocated
	}

	// Compute the number of port
	conf_desc_lgt = le16_to_cpu(dev->conf_desc->wTotalLength);
	ptr_iface = (usb_iface_desc_t*)dev->conf_desc;
	uhi_cdc_dev.nb_port = 0;
	while (conf_desc_lgt) {
		if ((ptr_iface->bDescriptorType == USB_DT_INTERFACE)
				&& (ptr_iface->bInterfaceClass == CDC_CLASS_COMM)
				&& (ptr_iface->bInterfaceSubClass == CDC_SUBCLASS_ACM)
				&& (ptr_iface->bInterfaceProtocol <= CDC_PROTOCOL_V25TER)) {
			// New COM port has been found
			uhi_cdc_dev.nb_port++;
		}
		Assert(conf_desc_lgt>=ptr_iface->bLength);
		conf_desc_lgt -= ptr_iface->bLength;
		ptr_iface = (usb_iface_desc_t*)((uint8_t*)ptr_iface + ptr_iface->bLength);
	}
	if (uhi_cdc_dev.nb_port == 0) {
		return UHC_ENUM_UNSUPPORTED; // No interface supported
	}

	// Alloc port structures
	uhi_cdc_dev.port = malloc(uhi_cdc_dev.nb_port * sizeof(uhi_cdc_port_t));
	if (uhi_cdc_dev.port == NULL) {
		Assert(false);
		return UHC_ENUM_SOFTWARE_LIMIT;
	}
	// Initialize structure
	for (i = 0; i<uhi_cdc_dev.nb_port; i++) {
		uhi_cdc_dev.port[i].ep_comm_in = 0;
		uhi_cdc_dev.port[i].iface_data = 0xFF;
		uhi_cdc_dev.port[i].ep_data_rx = 0;
		uhi_cdc_dev.port[i].ep_data_tx = 0;
	}

	// Fill port structures
	conf_desc_lgt = le16_to_cpu(dev->conf_desc->wTotalLength);
	ptr_iface = (usb_iface_desc_t*)dev->conf_desc;
	b_iface_comm = false;
	b_iface_data = false;
	port_num = 0;
	while (conf_desc_lgt) {
		switch (ptr_iface->bDescriptorType) {

		case USB_DT_INTERFACE:
			if ((ptr_iface->bInterfaceClass == CDC_CLASS_COMM)
					&& (ptr_iface->bInterfaceSubClass == CDC_SUBCLASS_ACM)
					&& (ptr_iface->bInterfaceProtocol <= CDC_PROTOCOL_V25TER) ) {
				// New Communication Class COM port has been found
				b_iface_comm = true;
				ptr_port = &uhi_cdc_dev.port[port_num++];
				ptr_port->iface_comm = ptr_iface->bInterfaceNumber;
			} else {
				// Stop allocation endpoint(s)
				b_iface_comm = false;
			}
			if ((ptr_iface->bInterfaceClass == CDC_CLASS_DATA)
					&& (ptr_iface->bInterfaceSubClass == 0)
					&& (ptr_iface->bInterfaceProtocol == 0) ) {
				for (i = 0; i<uhi_cdc_dev.nb_port; i++) {
					ptr_port = &uhi_cdc_dev.port[i];
					if (ptr_port->iface_data == 0xFF) {
						b_iface_data = true;
						break;
					}
					else if (ptr_port->iface_data == ptr_iface->bInterfaceNumber) {
						// New CDC DATA Class has been found
						// and correspond at a CDC COMM Class
						b_iface_data = true;
						break;
					}
				}
			} else {
				// Stop allocation endpoint(s)
				b_iface_data = false;
			}
			break;

		case CDC_CS_INTERFACE:
			if (!b_iface_comm) {
				break;
			}
			if (((usb_cdc_call_mgmt_desc_t*)ptr_iface)->bDescriptorSubtype == CDC_SCS_CALL_MGMT) {
				ptr_port->iface_data = ((usb_cdc_call_mgmt_desc_t*)ptr_iface)->bDataInterface;
			}
			break;

		case USB_DT_ENDPOINT:
			//  Allocation of the endpoint
			if (b_iface_comm) {
				Assert (((usb_ep_desc_t*)ptr_iface)->bmAttributes == USB_EP_TYPE_INTERRUPT);
				Assert (((usb_ep_desc_t*)ptr_iface)->bEndpointAddress & USB_EP_DIR_IN);
				if (!uhd_ep_alloc(dev->address, (usb_ep_desc_t*)ptr_iface)) {
					uhi_cdc_free_device();
					return UHC_ENUM_HARDWARE_LIMIT; // Endpoint allocation fail
				}
				ptr_port->ep_comm_in = ((usb_ep_desc_t*)ptr_iface)->bEndpointAddress;
			}
			if (b_iface_data) {
				Assert (((usb_ep_desc_t*)ptr_iface)->bmAttributes == USB_EP_TYPE_BULK);
				if (!uhd_ep_alloc(dev->address, (usb_ep_desc_t*)ptr_iface)) {
					uhi_cdc_free_device();
					return UHC_ENUM_HARDWARE_LIMIT; // Endpoint allocation fail
				}

				if (((usb_ep_desc_t*)ptr_iface)->bEndpointAddress & USB_EP_DIR_IN) {
					ptr_port->ep_data_rx = ((usb_ep_desc_t*)ptr_iface)->bEndpointAddress;
				} else {
					ptr_port->ep_data_tx = ((usb_ep_desc_t*)ptr_iface)->bEndpointAddress;
				}


			}
			break;

		}
		Assert(conf_desc_lgt >= ptr_iface->bLength);
		conf_desc_lgt -= ptr_iface->bLength;
		ptr_iface = (usb_iface_desc_t*)((uint8_t*)ptr_iface + ptr_iface->bLength);
	}

	// We don't need to support more than one port for now, let's ignore devices
	// that have more.
	if (uhi_cdc_dev.nb_port > 1) {
		return UHC_ENUM_UNSUPPORTED;
	}

	// Check installed ports
	for (i = 0; i < uhi_cdc_dev.nb_port; i++) {
		if ((uhi_cdc_dev.port[i].ep_comm_in == 0)
				|| (uhi_cdc_dev.port[i].ep_data_rx == 0)
				|| (uhi_cdc_dev.port[i].ep_data_tx == 0)) {
			// Install is not complete
			uhi_cdc_free_device();
			return UHC_ENUM_UNSUPPORTED;
		}
	}
	uhi_cdc_dev.b_enabled = false;
	uhi_cdc_dev.dev = dev;
	return UHC_ENUM_SUCCESS;
}

void uhi_cdc_enable(uhc_device_t* dev)
{
	if (uhi_cdc_dev.dev != dev) {
		return; // No interface to enable
	}
	uhi_cdc_dev.b_enabled = true;

	// Start all data transfers
	UHI_CDC_CHANGE(dev,true);
}

void uhi_cdc_uninstall(uhc_device_t* dev)
{
	if (uhi_cdc_dev.dev != dev) {
		return; // Device not enabled in this interface
	}
	uhi_cdc_dev.dev = NULL;
	uhi_cdc_free_device();
	UHI_CDC_CHANGE(dev,false);
}


/**
 * External API.
 */

bool uhi_cdc_open(uint8_t port, usb_cdc_line_coding_t *configuration)
{
	// Send configuration
	if (!uhi_cdc_set_conf(port, configuration)) {
		return false;
	}
	// Send DTR
	if (!uhi_cdc_set_ctrl_line(port, CDC_CTRL_SIGNAL_DTE_PRESENT)) {
		return false;
	}
	return true;
}

void uhi_cdc_close(uint8_t port)
{
	// Clear DTR
	uhi_cdc_set_ctrl_line(port, 0);
}

bool uhi_cdc_in_run(uint8_t port, uint8_t* buf, iram_size_t buf_size,
              uhd_callback_trans_t callback)
{
	uhi_cdc_port_t *ptr_port;
	// Select port
	ptr_port = uhi_cdc_get_port(port);
	if (ptr_port == NULL) {
		return false;
	}

	return uhd_ep_run(
		uhi_cdc_dev.dev->address,
		ptr_port->ep_data_rx,
		false,
		buf,
		buf_size,
		UHI_CDC_TIMEOUT,
		callback);
}

bool uhi_cdc_out_run(uint8_t port, uint8_t * buf, iram_size_t buf_size,
		      uhd_callback_trans_t callback)
{
	uhi_cdc_port_t *ptr_port;
	// Select port
	ptr_port = uhi_cdc_get_port(port);
	if (ptr_port == NULL) {
		return false;
	}

	return uhd_ep_run(
		uhi_cdc_dev.dev->address,
		ptr_port->ep_data_tx,
		true,
		buf,
		buf_size,
		UHI_CDC_TIMEOUT,
		callback);
}


/**
 * Internal function definitions.
 */
static void uhi_cdc_free_device(void)
{
	if (uhi_cdc_dev.port == NULL) {
		return;
	}

	free(uhi_cdc_dev.port);
}

static uhi_cdc_port_t* uhi_cdc_get_port(uint8_t port_num)
{
	if (uhi_cdc_dev.dev == NULL) {
		return NULL;
	}
	if (port_num >= uhi_cdc_dev.nb_port) {
		return NULL;
	}
	return &uhi_cdc_dev.port[port_num];
}

static bool uhi_cdc_set_conf(uint8_t port, usb_cdc_line_coding_t *configuration)
{
	uhi_cdc_port_t *ptr_port;
	usb_setup_req_t req;

	// Select port
	ptr_port = uhi_cdc_get_port(port);
	if (ptr_port == NULL) {
		return false;
	}
	memcpy(&ptr_port->conf, configuration, sizeof(usb_cdc_line_coding_t));

	// Enable configuration
	req.bmRequestType = USB_REQ_RECIP_INTERFACE | USB_REQ_TYPE_CLASS | USB_REQ_DIR_OUT;
	req.bRequest = USB_REQ_CDC_SET_LINE_CODING;
	req.wValue = 0;
	req.wIndex = ptr_port->iface_comm;
	req.wLength = sizeof(usb_cdc_line_coding_t);
	if (!uhd_setup_request(uhi_cdc_dev.dev->address,
			&req,
			(uint8_t *) &ptr_port->conf,
			sizeof(usb_cdc_line_coding_t),
			NULL, NULL)) {
		return false;
	}
	return true;
}

static bool uhi_cdc_set_ctrl_line(uint8_t port, le16_t wValue)
{
	uhi_cdc_port_t *ptr_port;
	usb_setup_req_t req;

	// Select port
	ptr_port = uhi_cdc_get_port(port);
	if (ptr_port == NULL) {
		return false;
	}

	// Enable configuration
	req.bmRequestType = USB_REQ_RECIP_INTERFACE | USB_REQ_TYPE_CLASS | USB_REQ_DIR_OUT;
	req.bRequest = USB_REQ_CDC_SET_CONTROL_LINE_STATE;
	req.wValue = wValue;
	req.wIndex = ptr_port->iface_comm;
	req.wLength = 0;
	if (!uhd_setup_request(uhi_cdc_dev.dev->address,
			&req,
			NULL,
			0,
			NULL, NULL)) {
		return false;
	}
	return true;
}
