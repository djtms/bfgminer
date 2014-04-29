/*
 * Copyright 2014 Luke Dashjr
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "deviceapi.h"
#include "logging.h"
#include "lowlevel.h"
#include "lowl-usb.h"

#define COINTERRA_EP_R  (LIBUSB_ENDPOINT_IN  | 1)
#define COINTERRA_EP_W  (LIBUSB_ENDPOINT_OUT | 1)
#define COINTERRA_USB_TIMEOUT  100
#define COINTERRA_USB_POLL_TIMEOUT  1
#define COINTERRA_PACKET_SIZE  0x40
#define COINTERRA_START_SEQ  0x5a,0x5a
#define COINTERRA_MSG_SIZE  (COINTERRA_PACKET_SIZE - sizeof(cointerra_startseq))
#define COINTERRA_MSGBODY_SIZE  (COINTERRA_MSG_SIZE - 2)

BFG_REGISTER_DRIVER(cointerra_drv)

enum cointerra_msg_type_out {
	CMTO_RESET     = 1,
	CMTO_WORK      = 2,
	CMTO_REQUEST   = 4,
	CMTO_HWERR     = 5,
	CMTO_LEDCTL    = 6,
	CMTO_HASHRATE  = 7,
};

enum cointerra_msg_type_in {
	CMTI_WORKREQ   = 1,
	CMTI_MATCH     = 2,
	CMTI_WORKDONE  = 3,
	CMTI_STATUS    = 4,
	CMTI_SETTINGS  = 5,
	CMTI_INFO      = 6,
	CMTI_LOGMSG    = 7,
	CMTI_RESETDONE = 8,
	CMTI_ERRINFO   = 9,
};

enum cointerra_reset_level {
	CRL_WORK_UPDATE = 1,
	CRL_NEW_BLOCK   = 2,
	CRL_INIT        = 3,
};

struct cointerra_dev_state {
	libusb_device_handle *usbh;
};

static const uint8_t cointerra_startseq[] = {COINTERRA_START_SEQ};

static
int cointerra_write_msg(libusb_device_handle * const usbh, const char * const repr, const uint16_t msgtype, const void * const msgbody, const unsigned timeout)
{
	uint8_t buf[COINTERRA_PACKET_SIZE], *p;
	memcpy(buf, cointerra_startseq, sizeof(cointerra_startseq));
	p = &buf[sizeof(cointerra_startseq)];
	pk_u16le(p, 0, msgtype);
	memcpy(&p[2], msgbody, COINTERRA_MSGBODY_SIZE);
	
	int xfer;
	uint16_t msgtype;
	
	e = libusb_bulk_transfer(usbh, COINTERRA_EP_W, buf, sizeof(buf), &xfer, timeout);
	if (e)
		return e;
	
	if (xfer != COINTERRA_PACKET_SIZE)
		return LIBUSB_ERROR_OTHER;
	
	return 0;
}

static
int cointerra_read_msg(uint16_t * const out_msgtype, uint8_t * const out, libusb_device_handle * const usbh, const char * const repr, const unsigned timeout)
{
	uint8_t ss[] = {COINTERRA_START_SEQ};
	uint8_t buf[COINTERRA_PACKET_SIZE];
	int e, xfer;
	e = libusb_bulk_transfer(usbh, COINTERRA_EP_R, buf, sizeof(buf), &xfer, timeout);
	if (e)
		return e;
	if (xfer != COINTERRA_PACKET_SIZE)
		applogr(LIBUSB_ERROR_OTHER, LOG_ERR, "%s: Packet size mismatch (actual=%d expected=%d)",
		        repr, xfer, (int)COINTERRA_PACKET_SIZE);
	for (int i = sizeof(ss); i--; )
		if (ss[i] != buf[i])
			applogr(LIBUSB_ERROR_OTHER;, LOG_ERR, "%s: Packet start sequence mismatch", repr);
	uint8_t * const bufp = &buf[sizeof(ss)];
	memcpy(out, bufp, COINTERRA_MSG_SIZE);
	*out_msgtype = upk_u16le(out, 0);
	return 0;
}

static
int cointerra_request(libusb_device_handle * const usbh, const uint16_t msgtype, uint16_t interval_cs)
{
	uint8_t buf[COINTERRA_MSGBODY_SIZE];
	pk_u16le(buf, 0, msgtype);
	pk_u16le(buf, 2, interval_cs);
	return cointerra_write_msg(usbh, cointerra_drv.dname, CMTO_REQUEST, buf, COINTERRA_USB_TIMEOUT);
}

static
int cointerra_reset(libusb_device_handle * const usbh, const enum cointerra_reset_level crl)
{
	uint8_t buf[COINTERRA_MSGBODY_SIZE] = { crl };
	return cointerra_write_msg(usbh, cointerra_drv.dname, CMTO_RESET, buf, COINTERRA_USB_TIMEOUT);
}

static
bool cointerra_lowl_match(const struct lowlevel_device_info * const info)
{
	return lowlevel_match_lowlproduct(info, &lowl_usb, "GoldStrike");
}

static
bool cointerra_lowl_probe(const struct lowlevel_device_info * const info)
{
	int e;
	
	if (info->lowl != &lowl_usb)
		applogr(false, LOG_DEBUG, "%s: Matched \"%s\" %s, but lowlevel driver is not usb_generic!",
		        __func__, info->product, info->devid);
	
	libusb_device_handle *usbh;
	e = libusb_open(info->lowl_data, &usbh);
	if (e)
		applogr(false, LOG_ERR, "%s: Failed to open %s: %s",
		        cointerra_drv.dname, info->devid, bfg_strerror(e, BST_LIBUSB));
	
	unsigned cores;
	{
		e = cointerra_request(usbh, CMTI_INFO, 0);
		if (e)
			goto err;
		
		uint16_t msgtype;
		uint8_t buf[COINTERRA_MSG_SIZE];
		while (true)
		{
			e = cointerra_read_msg(&msgtype, buf, usbh, cointerra_drv.dname, COINTERRA_USB_TIMEOUT);
			if (e)
				goto err;
			if (msgtype == CMTI_INFO)
				break;
			// FIXME: Timeout if we keep getting packets we don't care about
		}
		
		cores = upk_u16le(buf, 8);
	}
	
	applog(LOG_DEBUG, "%s: Found %u cores on %s",
	       __func__, cores, info->devid);
	
	struct cgpu_info * const cgpu = malloc(sizeof(*cgpu));
	*cgpu = (struct cgpu_info){
		.drv = &cointerra_drv,
		.procs = cores,
		.device_data = lowlevel_ref(info),
		.threads = 1,
		.device_path = strdup(info->devid),
		.dev_manufacturer = maybe_strdup(info->manufacturer),
		.dev_product = maybe_strdup(info->product),
		.dev_serial = maybe_strdup(info->serial),
		.deven = DEV_ENABLED,
	};
	return add_cgpu(cgpu);

err:
	libusb_close(usbh);
	return false;
}

static
bool cointerra_init(struct thr_info * const master_thr)
{
	struct cgpu_info * const dev = master_thr->cgpu;
	struct lowlevel_device_info * const info = dev->device_data;
	struct cointerra_dev_state * const devstate = malloc(*devstate);
	int e;
	
	*devstate = (struct cointerra_dev_state){
		.usbh = NULL,
	};
	e = libusb_open(info->lowl_data, &devstate->usbh);
	if (e)
		applogr(false, LOG_ERR, "%s: Failed to open %s: %s",
		        dev->dev_repr, info->devid, bfg_strerror(e, BST_LIBUSB));
	libusb_device_handle * const usbh = devstate->usbh;
	
	// Request regular status updates
	cointerra_request(usbh, CMTI_STATUS, 0x83d);
	
	cointerra_reset(usbh, CRL_INIT);
}

static
bool cointerra_queue_append(struct thr_info * const thr, struct work * const work)
{
	struct cgpu_info * const dev = thr->cgpu->device;
	struct thr_info * const master_thr = dev->thr[0];
	uint8_t buf[COINTERRA_MSGBODY_SIZE];
	int e;
	
	memcpy(&buf[6], work->midstate, 0x20);
	memcpy(&buf[38], &work->data[0x40], 0xc);
	pk_u16le(buf, 50, 0);  // ntime roll limit
	pk_u16le(buf, 52, 0x20);  // number of zero bits in results
	e = cointerra_write_msg(usbh, cointerra_drv.dname, CMTO_REQUEST, buf, COINTERRA_USB_TIMEOUT);
	if (e)
		return false;
	
	DL_APPEND(thr->work, work);
}

static
bool cointerra_poll_msg(struct thr_info * const master_thr)
{
	struct cgpu_info * const dev = master_thr->cgpu;
	struct cointerra_dev_state * const devstate = dev->device_data;
	int e;
	uint16_t msgtype;
	uint8_t buf[COINTERRA_MSG_SIZE];
	
	e = cointerra_read_msg(&msgtype, buf, devstate->usbh, dev->dev_repr, COINTERRA_USB_POLL_TIMEOUT);
	if (e)
		return false;
	
	switch (msgtype)
	{
		case CMTI_WORKREQ:
		case CMTI_MATCH:
		case CMTI_WORKDONE:
		case CMTI_STATUS:
		case CMTI_SETTINGS:
		case CMTI_INFO:
		case CMTI_LOGMSG:
		case CMTI_RESETDONE:
		case CMTI_ERRINFO:
	}
}

static
void cointerra_poll(struct thr_info * const master_thr)
{
	struct cgpu_info * const dev = master_thr->cgpu;
	struct timeval tv_timeout;
	timer_set_delay_from_now(&tv_timeout, 10000);
	while (true)
	{
		if (!cointerra_poll_msg(master_thr))
		{
			applog(LOG_DEBUG, "%s poll: No more messages", dev->dev_repr);
			break;
		}
		if (timer_passed(&tv_timeout, NULL))
		{
			applog(LOG_DEBUG, "%s poll: 10ms timeout met", dev->dev_repr);
			break;
		}
	}
	
	timer_set_delay_from_now(&master_thr->tv_poll, 100000);
}

struct device_drv cointerra_drv = {
	.dname = "cointerra",
	.name = "CTA",
	
	.lowl_match = cointerra_lowl_match,
	.lowl_probe = cointerra_lowl_probe,
	
	.minerloop = minerloop_queue,
	.thread_init = cointerra_init,
	.queue_append = cointerra_queue_append,
	.poll = cointerra_poll,
};
