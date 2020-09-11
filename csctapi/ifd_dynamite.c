/*
        ifd_dynamite.c
        This module provides IFD handling functions for Smartmouse/Phoenix reader.
*/

#include "../globals.h"

#ifdef CARDREADER_DYNAMITE
#include "../oscam-time.h"
#include "icc_async.h"
#include "ifd_db2com.h"
#include "ifd_phoenix.h"
#include "io_serial.h"

#define OK 0
#define ERROR 1

#define GPIO_PIN (1 << (reader->detect - 4))

typedef enum {
	NOFW		= 0,
	READY		= 1,
	VEND_AX		= 2,
	START		= 3,
	MOUSE_PHOENIX	= 4,
	PHOENIX_357	= 5,
	PHOENIX_368	= 6,
	PHOENIX_400	= 7,
	PHOENIX_600	= 8,
	SMARTMOUSE_357	= 9,
	SMARTMOUSE_368	= 10,
	SMARTMOUSE_400	= 11,
	SMARTMOUSE_600	= 12,
	CARDPROGRAMMER	= 13,
} dynamite_device_status_t;

static const char *dynamite_device_status[] = {
	"nofw",
	"ready",
	"vend_ax",
	"start",
	"mouse_phoenix",
	"phoenix357",
	"phoenix368",
	"phoenix400",
	"phoenix600",
	"smartmouse357",
	"smartmouse368",
	"smartmouse400",
	"smartmopuse600",
	"cardprogrammer",
};

typedef enum {
	NONE_DEVICE = 0,
	DYNAMITE_DEVICE = 1,
	DYNAMITE_PLUS_DEVICE = 2,
	DYNAMITE_TINY_DEVICE = 3,
} dynamite_device_list_t;

static const char *dynamite_device_list[] = {
	"nodevice",
	"dynamite",
	"dynamiteplus",
	"dynamitetiny",
};

typedef enum {
	IOCTL_SET_PHOENIX_357 = 0x000000c1,
	IOCTL_SET_PHOENIX_368 = 0x000000c2,
	IOCTL_SET_PHOENIX_400 = 0x000000c3,
	IOCTL_SET_PHOENIX_600 = 0x000000c4,
	IOCTL_SET_SMARTMOUSE_357 = 0x000000c5,
	IOCTL_SET_SMARTMOUSE_368 = 0x000000c6,
	IOCTL_SET_SMARTMOUSE_400 = 0x000000c7,
	IOCTL_SET_SMARTMOUSE_600 = 0x000000c8,
	IOCTL_SET_CARDPROGRAMMER = 0x00000c10,
	IOCTL_SEND_BULK_COMMAND = 0x00000c11,
	IOCTL_RECV_BULK_COMMAND = 0x00000c12,
	IOCTL_SEND_VENDOR_COMMAND = 0x00000c13,
	IOCTL_RECV_VENDOR_COMMAND = 0x00000c14,
	IOCTL_DEVICE_INFORMATION_COMMAND = 0x00000c15,
} _dynamite_ioctl_command_t;

struct dynamite_device_information_command {
	int device;
	int status;
	int vid;
	int pid;
};

#define DYNAMITE_DEVICE "/dev/dynamite_programmer"

#define DYNAMITE_WRITE_DELAY 100000
#define DYNAMITE_READ_DELAY 200000

static inline int reader_use_gpio(struct s_reader *reader)
{
	return reader->use_gpio && reader->detect > 4;
}

static void set_gpio(struct s_reader *reader, int32_t level)
{
	int ret = 0;

	ret |= read(reader->gpio_outen, &reader->gpio, sizeof(reader->gpio));
	reader->gpio |= GPIO_PIN;
	ret |= write(reader->gpio_outen, &reader->gpio, sizeof(reader->gpio));

	ret |= read(reader->gpio_out, &reader->gpio, sizeof(reader->gpio));
	if(level > 0)
		{ reader->gpio |= GPIO_PIN; }
	else
		{ reader->gpio &= ~GPIO_PIN; }
	ret |= write(reader->gpio_out, &reader->gpio, sizeof(reader->gpio));

	rdr_log_dbg(reader, D_IFD, "%s level: %d ret: %d", __func__, level, ret);
}

static void set_gpio_input(struct s_reader *reader)
{
	int ret = 0;
	ret |= read(reader->gpio_outen, &reader->gpio, sizeof(reader->gpio));
	reader->gpio &= ~GPIO_PIN;
	ret |= write(reader->gpio_outen, &reader->gpio, sizeof(reader->gpio));
	rdr_log_dbg(reader, D_IFD, "%s ret:%d", __func__, ret);
}

static int32_t get_gpio(struct s_reader *reader)
{
	int ret = 0;
	set_gpio_input(reader);
	ret = read(reader->gpio_in, &reader->gpio, sizeof(reader->gpio));
	rdr_log_dbg(reader, D_IFD, "%s ok:%d ret:%d", __func__, reader->gpio & GPIO_PIN, ret);
	if(reader->gpio & GPIO_PIN)
		{ return OK; }
	else
		{ return ERROR; }
}

int32_t Dynamite_Set_Clock(struct s_reader *reader, unsigned int mhz)
{
	struct dynamite_device_information_command dynamite_info_cmd;

	rdr_log_dbg(reader, D_IFD, "Setting Smartcard clock at: %d", mhz);

	int fd = open(DYNAMITE_DEVICE, O_RDWR);
	if (fd < 0) {
		rdr_log(reader, "ERROR: Opening device %s (errno=%d %s)",
			DYNAMITE_DEVICE, errno, strerror(errno));
		return ERROR;
	}

	if (ioctl(fd, IOCTL_DEVICE_INFORMATION_COMMAND, &dynamite_info_cmd) < 0)
	{
		rdr_log(reader, "ERROR: Send ioctl command to device %s (errno=%d %s)",
			DYNAMITE_DEVICE, errno, strerror(errno));
		return ERROR;
	}
	rdr_log(reader, "Found device: %s, vid: 0x%04x, pid: 0x%04x, status: %s", dynamite_device_list[dynamite_info_cmd.device], dynamite_info_cmd.vid, dynamite_info_cmd.pid, dynamite_device_status[dynamite_info_cmd.status]);

	//DTR down
	IO_Serial_DTR_Set(reader);

	if(mhz == 357)
	{
		if (dynamite_info_cmd.status != PHOENIX_357) {
			rdr_log(reader, "%s: Using oscillator 1 (3.57MHz)", __func__);
			if (ioctl(fd, IOCTL_SET_PHOENIX_357) < 0) {
				rdr_log(reader, "ERROR: Send ioctl command to device %s (errno=%d %s)",
					DYNAMITE_DEVICE, errno, strerror(errno));
				return ERROR;
			}
		}
	}
	else if(mhz == 368)
	{
		if (dynamite_info_cmd.status != PHOENIX_368) {
			rdr_log(reader, "%s: Using oscillator 2 (3.68MHz)", __func__);
			if (ioctl(fd, IOCTL_SET_PHOENIX_368) < 0) {
				rdr_log(reader, "ERROR: Send ioctl command to device %s (errno=%d %s)",
					DYNAMITE_DEVICE, errno, strerror(errno));
				return ERROR;
			}
		}
	}
	else if(mhz == 400)
	{
		if (dynamite_info_cmd.status != PHOENIX_400) {
			rdr_log(reader, "%s: Using oscillator 3 (4.00MHz)", __func__);
			if (ioctl(fd, IOCTL_SET_PHOENIX_400) < 0) {
				rdr_log(reader, "ERROR: Send ioctl command to device %s (errno=%d %s)",
					DYNAMITE_DEVICE, errno, strerror(errno));
				return ERROR;
			}
		}
	}
	else if(mhz == 600)
	{
		if (dynamite_info_cmd.status != PHOENIX_600) {
			rdr_log(reader, "%s: Using oscillator 4 (6.00MHz)", __func__);
			if (ioctl(fd, IOCTL_SET_PHOENIX_600) < 0) {
				rdr_log(reader, "ERROR: Send ioctl command to device %s (errno=%d %s)",
					DYNAMITE_DEVICE, errno, strerror(errno));
				return ERROR;
			}
		}
	}
	else
	{
		rdr_log(reader, "%s: Dynamite support only mhz=357, mhz=368 mhz=400 or mhz=600", __func__);
		rdr_log(reader, "%s: Forced oscillator 1 (3.57MHz)", __func__);
		if (ioctl(fd, IOCTL_SET_PHOENIX_357) < 0) {
			rdr_log(reader, "ERROR: Send ioctl command to device %s (errno=%d %s)",
				DYNAMITE_DEVICE, errno, strerror(errno));
			return ERROR;
		}
		reader->mhz = 357;
		reader->cardmhz = 357;
	}

	unsigned char dat[ATR_MAX_SIZE];
	int32_t n = 0;
	while(n < 2 && !IO_Serial_Read(reader, 0, 300000, 1, dat + n))
		{ n++; }

	//DTR up
	IO_Serial_DTR_Clr(reader);
	rdr_log_dbg(reader, D_IFD, "Smartcard clock at %d set", mhz);
	return OK;
}

int32_t Dynamite_Init(struct s_reader *reader)
{
	const struct s_cardreader *crdr_ops = reader->crdr;
	if (!crdr_ops) return ERROR;

	if(crdr_ops->flush) { IO_Serial_Flush(reader); }

	// define reader->gpio number used for card detect and reset. ref to globals.h
	if(reader_use_gpio(reader))
	{
		reader->gpio_outen = open("/dev/gpio/outen", O_RDWR);
		reader->gpio_out   = open("/dev/gpio/out",   O_RDWR);
		reader->gpio_in    = open("/dev/gpio/in",    O_RDWR);
		rdr_log_dbg(reader, D_IFD, "init gpio_outen:%d gpio_out:%d gpio_in:%d",
					   reader->gpio_outen, reader->gpio_out, reader->gpio_in);
		set_gpio_input(reader);
	}

	rdr_log_dbg(reader, D_IFD, "Initializing reader type=%d", reader->typ);

	/* Default serial port settings */
	if(reader->atr[0] == 0)
	{
		if(IO_Serial_SetParams(reader, DEFAULT_BAUDRATE, 8, PARITY_EVEN, 2, NULL, NULL)) { return ERROR; }
		if(crdr_ops->flush) { IO_Serial_Flush(reader); }
	}
	return OK;
}

int32_t Dynamite_GetStatus(struct s_reader *reader, int32_t *status)
{
	// detect card via defined reader->gpio
	if(reader_use_gpio(reader))
	{
		*status = !get_gpio(reader);
		return OK;
	}
	else
	{
		return IO_Serial_GetStatus(reader, status);
	}
}

int32_t Dynamite_Reset(struct s_reader *reader, ATR *atr)
{
	rdr_log_dbg(reader, D_IFD, "Resetting card");
	int32_t ret;
	int32_t i;
	unsigned char buf[ATR_MAX_SIZE];
	int32_t parity[3] = {PARITY_EVEN, PARITY_ODD, PARITY_NONE};

	call(IO_Serial_SetBaudrate(reader, DEFAULT_BAUDRATE));

	const struct s_cardreader *crdr_ops = reader->crdr;
	if (!crdr_ops) return ERROR;

	for(i = 0; i < 3; i++)
	{
		if(crdr_ops->flush) { IO_Serial_Flush(reader); }
		if(crdr_ops->set_parity) { IO_Serial_SetParity(reader, parity[i]); }

		ret = ERROR;

		IO_Serial_Ioctl_Lock(reader, 1);
		if(reader_use_gpio(reader))
			{ set_gpio(reader, 0); }
		else
			{ IO_Serial_RTS_Set(reader); }

		cs_sleepms(50);

		// felix: set card reset hi (inactive)
		if(reader_use_gpio(reader))
			{ set_gpio_input(reader); }
		else
			{ IO_Serial_RTS_Clr(reader); }
		cs_sleepms(50);
		IO_Serial_Ioctl_Lock(reader, 0);

		int32_t n = 0;
		while(n < ATR_MAX_SIZE && !IO_Serial_Read(reader, 0, ATR_TIMEOUT, 1, buf + n))
			{ n++; }
		if(n == 0)
			{ continue; }
		if(ATR_InitFromArray(atr, buf, n) != ERROR)
			{ ret = OK; }
		// Succesfully retrieve ATR
		if(ret == OK)
			{ break; }
	}

	return ret;
}

int32_t Dynamite_Close(struct s_reader *reader)
{
	rdr_log_dbg(reader, D_IFD, "Closing Dynamite device %s", reader->device);
	if(reader_use_gpio(reader))
	{
		if(reader->gpio_outen > -1)
			{ close(reader->gpio_outen); }
		if(reader->gpio_out > -1)
			{ close(reader->gpio_out); }
		if(reader->gpio_in > -1)
			{ close(reader->gpio_in); }
	}
	IO_Serial_Close(reader);
	return OK;
}

static int32_t dynamite_init(struct s_reader *reader)
{
	const struct s_cardreader *crdr_ops = reader->crdr;
	if (!crdr_ops) return ERROR;

	if(detect_db2com_reader(reader))
	{
		reader->crdr = crdr_ops = &cardreader_db2com;
		return crdr_ops->reader_init(reader);
	}

	reader->handle = open(reader->device,  O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(reader->handle < 0)
	{
		rdr_log(reader, "ERROR: Opening device %s (errno=%d %s)",
				reader->device, errno, strerror(errno));
		return ERROR;
	}
	if(Dynamite_Init(reader))
	{
		rdr_log(reader, "ERROR: Dynamite_Init returns error");
		Dynamite_Close(reader);
		return ERROR;
	}

	// SET CLOCK
	cs_sleepms(200);
	Dynamite_Set_Clock(reader, reader->mhz);

	return OK;
}

const struct s_cardreader cardreader_dynamite =
{
	.desc          = "dynamite",
	.typ           = R_MOUSE,
	.flush         = 1,
	.read_written  = 1,
	.need_inverse  = 1,
	.reader_init   = dynamite_init,
	.get_status    = Dynamite_GetStatus,
	.activate      = Dynamite_Reset,
	.transmit      = IO_Serial_Transmit,
	.receive       = IO_Serial_Receive,
	.close         = Dynamite_Close,
	.set_parity    = IO_Serial_SetParity,
	.set_baudrate  = IO_Serial_SetBaudrate,
};

#endif
