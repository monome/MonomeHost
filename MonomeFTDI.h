/*
  Copyright (c) 2014 by Ezra Buchla.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 
  this code is indebted to the following:

  - USBHost library by Arduino
  https://github.com/arduino/Arduino

  - arduino USB Host shield by circuitsathome
  https://github.com/felis/USB_Host_Shield_2.0

  - nw2s/b library by scott wilson 
  https://github.com/nw2s/b

  - monome device driver for aleph, by ezra buchla and tehn
  https://github.com/tehn/aleph/

  - monome protocol by tehn
  http://monome.org/docs/tech:serial

*/


#ifndef __MONOME_HOST__MONOME_FTDI_H_
#define __MONOME_HOST__MONOME_FTDI_H_

#include <stdint.h>
#include "usb_ch9.h"
#include "Usb.h"
#include "Arduino.h"
#include "confdescparser.h"

//--- defines
// FIXME: many of these could be const class variables

////////////
/// dbg
//#define PRINT_DBG(x) Serial.print((x))
#define PRINT_DBG(x)
///////////


#define bmREQ_FTDI_OUT  0x40
#define bmREQ_FTDI_IN   0xc0
#define FTDI_SIO_SET_BAUD_RATE 3
#define FTDI_SIO_SET_FLOW_CTRL 2
#define FTDI_SIO_DISABLE_FLOW_CTRL 0x0

// Only single port chips are currently supported by the library,
// so only three endpoints are allocated.
#define FTDI_MAX_ENDPOINTS	3

/// maximum unpacked size of led buffer,
/// all frames/elements, 1 byte per led.
// arc4 and 256 are tied.
#define MONOME_LED_BUF_BYTES 256

// how many leds on each side of a quad
#define MONOME_QUAD_LEDS 8

// how many leds in 1 row of the buffer
#define MONOME_LED_ROW_BYTES 16
// define mul as lshift
#define MONOME_LED_ROW_LS 4

//-- sizes for the actual data to be send over usb
// one frame is an 8x8 quadrant; 256 has 4.
#define MONOME_GRID_MAX_FRAMES 4

// map stores binary data for each led in 8x8
#define MONOME_GRID_MAP_BYTES 8

// a frame is one knob, max is arc4
#define MONOME_RING_MAX_FRAMES 4

// manufacturer string length
#define MONOME_MANSTR_LEN 6
// product string lengthextern 
#define MONOME_PRODSTR_LEN 8
// serial string length
#define MONOME_SERSTR_LEN 9

// tx buffer length
#define MONOME_TX_BUF_LEN 72

// rx buffer length
#define MONOME_RX_BUF_LEN 64

// fwd declare
class MonomeFtdi;

// abstract base class for parsers
class MonomeReportParser 
{
 public:
  virtual uint8_t CheckDeviceDesc(char* man, char* prod, char* ser) = 0;
  //  virtual void Parse(MonomeFtdi* ftdi, uint32_t len, uint8_t *buf) = 0;
  //// FIXME: bad form here. 
  //// parser is assumed to have access to a driver instance.
  //// would be better to pass the instance/data as indicated above.
  virtual void Parse(void) = 0;
};
  

class MonomeFtdi : public USBDeviceConfig, public UsbConfigXtracter
{
 protected:

  typedef struct 
  {
    uint32_t dwDTERate; 
    uint8_t bCharFormat; 
    uint8_t bParityType; 
    uint8_t bDataBits;
  } LineCoding;

  static const uint32_t epDataInIndex;	// DataIn endpoint index
  static const uint32_t epDataOutIndex;	// DataOUT endpoint index

  // Mandatory members 
  USBHost	*pUsb;
  uint32_t	bAddress;	// Device USB address
  uint32_t	bConfNum;	// configuration number
  uint8_t 	bControlIface; 	// Control interface value
  uint32_t	bNumEP;		// total number of EP in the configuration
  uint32_t 	qNextPollTime; // next poll time
  bool	bPollEnable; // poll enable flag

  // Endpoint data structure describing the device EP 
  EpInfo	epInfo[MAX_ENDPOINTS];
  // polling period in milliseconds
  uint32_t 	pollPeriod;
  // flag when device ID is finished
  bool	bNeedsId;

  // pointer to controller
  MonomeReportParser* pController;
  // receive buffer
  uint8_t rxBuf[MONOME_RX_BUF_LEN];
  // receive byte count
  uint32_t rxBytes;

 public:
  MonomeFtdi(USBHost &usb, MonomeReportParser *con);

  // Basic IO 
  //  uint32_t read(uint32_t *nreadbytes, uint8_t *dataptr);
  // always use internal RX butter
  uint32_t read(void);
  // always use external TX buffer
  uint32_t write(uint32_t datalen, uint8_t *dataptr);

  // Line control setup 
  uint8_t setLineCoding(const LineCoding *dataptr);
  uint8_t setControlLineState(uint8_t state);

  void setPollPeriod(uint32_t period) { pollPeriod = period; }

  // getters for rx count / data
  uint32_t rx_bytes(void) { return rxBytes > 1 ? rxBytes - 2 : 0; }
  uint8_t* rx_buf(void) { return rxBuf + 2; }

  // string descriptor utility
  uint32_t string_desc(uint8_t idx, uint8_t* data);

  // USBDeviceConfig implementation 
  virtual uint32_t Init(uint32_t parent, uint32_t port, uint32_t lowspeed);
  virtual uint32_t Release();
  virtual uint32_t Poll();

  virtual uint32_t GetAddress() { return bAddress; };

  // UsbConfigXtracter implementation 
  virtual void EndpointXtract(uint32_t conf, uint32_t iface, uint32_t alt, uint32_t proto, const USB_ENDPOINT_DESCRIPTOR *ep);
};

#endif
