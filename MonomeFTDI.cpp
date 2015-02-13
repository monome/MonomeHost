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


#include <stdint.h>

#include "Usb.h"
#include "MonomeFTDI.h"

#define LOBYTE(x) ((char*)(&(x)))[0]
#define HIBYTE(x) ((char*)(&(x)))[1]

const uint32_t MonomeFtdi::epDataInIndex  = 1;
const uint32_t MonomeFtdi::epDataOutIndex = 2;

MonomeFtdi::MonomeFtdi(USBHost &usb, MonomeReportParser *con) : 
  bAddress(0), 
  bNumEP(1),
  pollPeriod(1), // polling time default set here
  bPollEnable(0),
  pController(con),
  rxBytes(0),
  bNeedsId(1)
{
  this->pUsb = &usb;
  memset(rxBuf, 0, MONOME_RX_BUF_LEN);
	
  // Setup an empty set of endpoints 
  for (uint32_t i = 0; i < MAX_ENDPOINTS; ++i)
    {
      epInfo[i].deviceEpNum	= 0;
      epInfo[i].hostPipeNum	= 0;
      epInfo[i].maxPktSize	= (i) ? 0 : 8;
      epInfo[i].epAttribs		= 0;
      epInfo[i].bmNakPower  	= (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
    }

  // Register ourselves in USB subsystem 
  if (pUsb)
    {
      pUsb->RegisterDeviceClass(this);
    }
}

// string descriptor utility
uint32_t MonomeFtdi::string_desc(uint8_t idx, uint8_t* buf) {
  //    uint8_t buf[66];
  uint32_t rcode;
  uint8_t len;
  uint8_t i;
  uint32_t langid;
  // get language table length
  rcode = pUsb->getStrDescr( bAddress, 0, 1, 0, 0, buf );  
  if( rcode ) {
    PRINT_DBG("\r\n Error retrieving LangID table length");
    return( rcode );
  }
  // length is the first byte
  len = buf[ 0 ];      
  // get language table
  rcode = pUsb->getStrDescr( bAddress, 0, len, 0, 0, buf );  
  if( rcode ) {
    PRINT_DBG("\r\n Error retrieving LangID table");
    return( rcode );
  }
  //get first langid
  HIBYTE( langid ) = buf[ 3 ];                            
  LOBYTE( langid ) = buf[ 2 ];
  rcode = pUsb->getStrDescr( bAddress, 0, 1, idx, langid, buf );
  if( rcode ) {
    PRINT_DBG("\r\n Error retrieving string length");
    return( rcode );
  }
  // get the string contents
  len = buf[ 0 ];
  rcode = pUsb->getStrDescr( bAddress, 0, len, idx, langid, buf );
  if( rcode ) {
    PRINT_DBG("\r\n Error retrieving string");
    return( rcode );
  }

  for( i = 2; i < len; i+=2 ) {
    PRINT_DBG( buf[i] );
    PRINT_DBG(" ");
  }
  PRINT_DBG("\r\n ");

  for( i = 2; i < len; i+=2 ) {
    PRINT_DBG( (char)buf[i] );
    PRINT_DBG(" ");
  }
  PRINT_DBG("\r\n ");

  return 0;
}


uint32_t MonomeFtdi::Init(uint32_t parent, uint32_t port, uint32_t lowspeed)
{
  uint8_t 	buf[sizeof(USB_DEVICE_DESCRIPTOR)];
  uint32_t 	rcode = 0;
  UsbDevice	*p = NULL;
  EpInfo	*oldep_ptr = NULL;
  uint32_t 	adkproto = -1;
  uint32_t	num_of_conf = 0;

  USB_DEVICE_DESCRIPTOR * udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR*>(buf);

  /// FIXME: surely this string size is overkill?
  uint8_t manStr[64];
  uint8_t prodStr[64];
  uint8_t serStr[64];

  // Get memory address of USB device address pool 
  AddressPool	&addrPool = pUsb->GetAddressPool();

  PRINT_DBG("\r\n initializing monome ftdi driver...");

  // Check if address has already been assigned to an instance 
  if (bAddress)
    {
      PRINT_DBG("\r\n USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE");
      return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
    }

  // Get pointer to pseudo device with address 0 assigned 
  p = addrPool.GetUsbDevicePtr(0);

  if (!p)
    {
      PRINT_DBG("\r\n USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL");
      return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
    }

  if (!p->epinfo)
    {
      PRINT_DBG("\r\n USB_ERROR_EPINFO_IS_NULL");
      return USB_ERROR_EPINFO_IS_NULL;
    }

  // Save old pointer to EP_RECORD of address 0 
  oldep_ptr = p->epinfo;

  // Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence 
  p->epinfo = epInfo;

  p->lowspeed = lowspeed;

  // Get device descriptor 
  rcode = pUsb->getDevDescr(0, 0, sizeof(USB_DEVICE_DESCRIPTOR), (uint8_t*)buf);

  // Restore p->epinfo 
  p->epinfo = oldep_ptr;

  if (rcode)
    {
      PRINT_DBG("Failed to get device descriptor : ");
      PRINT_DBG(rcode);
      PRINT_DBG("\r\n");
      return rcode;
    }

  // Allocate new address according to device class 
  bAddress = addrPool.AllocAddress(parent, false, port);

  // Extract Max Packet Size from device descriptor 
  epInfo[0].maxPktSize = (uint8_t)udd->bMaxPacketSize0;

  // Assign new address to the device 
  rcode = pUsb->setAddr(0, 0, bAddress);

  if (rcode)
    {
      p->lowspeed = false;
      addrPool.FreeAddress(bAddress);
      bAddress = 0;
      PRINT_DBG("setAddr failed with rcode ");
      PRINT_DBG(rcode);
      PRINT_DBG("\r\n");
      return rcode;
    }

  PRINT_DBG("device address is now ");
  PRINT_DBG(bAddress);
  PRINT_DBG("\r\n");

  p->lowspeed = false;

  // get pointer to assigned address record 
  p = addrPool.GetUsbDevicePtr(bAddress);

  if (!p)
    {
      PRINT_DBG("\r\n USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL");
      return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
    }

  p->lowspeed = lowspeed;

  // Assign epInfo to epinfo pointer - only EP0 is known 
  rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
  if (rcode)
    {
      PRINT_DBG("Failed setEpInfoEntry: ");
      PRINT_DBG(rcode);
      PRINT_DBG("\r\n");
      return rcode;
    }

  // Go through configurations, find first bulk-IN, bulk-OUT EP, fill epInfo and quit 
  num_of_conf = udd->bNumConfigurations;

  for (uint32_t i = 0; i < num_of_conf; ++i)
    {
      ConfigDescParser<0, 0, 0, 0> confDescrParser(this);

      delay(1);
      rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);

      if (rcode)
	{
	  PRINT_DBG("Failed to get configurarion descriptors: ");
	  PRINT_DBG(rcode);
	  PRINT_DBG("\r\n");
	  return rcode;
	}

      ////////
      ///// FIXME: (?) 
      /// i guess this stuff is to support some kind of clone ... 

      // monome FTDI only looks for 2. CDC ACM (leonardo) looks for 3  

      /// device type in controlller..
      // if (deviceType == DEVICE_40H_TRELLIS && bNumEP > 2)
      // 	{
      // 	  break;
      // 	}
      // else if (deviceType == DEVICE_SERIES && bNumEP > 1)
      // 	{
      // 	  break;
      // 	}
    }

  // if (bNumEP == 3)
  // {
  // Assign epInfo to epinfo pointer - this time all 3 endpoins 
  rcode = pUsb->setEpInfoEntry(bAddress, 3, epInfo);
  if (rcode)
    {
      PRINT_DBG("Failed setEpInfoEntry: ");
      PRINT_DBG(rcode);
      PRINT_DBG("\r\n");
    }
  // }

  // Set Configuration Value 
  rcode = pUsb->setConf(bAddress, 0, bConfNum);

  if (rcode)
    {
      PRINT_DBG("setConf failed: ");
      PRINT_DBG(rcode);
      PRINT_DBG("\r\n");
      return rcode;
    }

  // Assume for now that the control interface is 0 
  bControlIface = 0;


  // Set the baud rate 
  uint16_t baud_value = 0;
  uint16_t baud_index = 0;
		
  uint32_t divisor3 = 48000000 / 2 / 115200; // divisor shifted 3 bits to the left

  static const unsigned char divfrac [8] = {0, 3, 2, 0, 1, 1, 2, 3};
  static const unsigned char divindex[8] = {0, 0, 0, 1, 0, 1, 1, 1};

  baud_value = divisor3 >> 3;
  baud_value |= divfrac [divisor3 & 0x7] << 14;
  baud_index = divindex[divisor3 & 0x7];

  // Deal with special cases for highest baud rates. 
  if (baud_value == 1) 
    {
      baud_value = 0;
    }
  else if (baud_value == 0x4001) 
    {
      baud_value = 1;
    }

  rcode = pUsb->ctrlReq(bAddress, 0, bmREQ_FTDI_OUT, FTDI_SIO_SET_BAUD_RATE, baud_value & 0xff, baud_value >> 8, baud_index, 0, 0, NULL, NULL);
		
  if (rcode)
    {
      PRINT_DBG("\r\n Error setting baudrate");
      return rcode;
    }

  // Set no flow control 
  rcode = pUsb->ctrlReq(bAddress, 0, bmREQ_FTDI_OUT, FTDI_SIO_SET_FLOW_CTRL, 0x11, 0x13, FTDI_SIO_DISABLE_FLOW_CTRL << 8, 0, 0, NULL, NULL);		


  /////////////////////////////////////
  // get device description and notify the controller
  rcode = string_desc( udd->iManufacturer, manStr);
  if (rcode) {
    PRINT_DBG("\r\n error getting man desc");
    return rcode;
  }
  rcode = string_desc( udd->iProduct, prodStr);
  if (rcode) {
    PRINT_DBG("\r\n error getting prod desc");
    return rcode;
  }
  rcode = string_desc( udd->iSerialNumber, serStr);
  if (rcode) {
    PRINT_DBG("\r\n error getting ser desc");
    return rcode;
  }
  // first 2 bytes are length and langid, or whatever. awesome
  // pController->check_device_desc(
  pController->CheckDeviceDesc(
			       (char*)(manStr + 2),
			       (char*)(prodStr + 2), 
			       (char*)(serStr + 2 ) );

  PRINT_DBG("\r\n \r\n Monome FTDI configured.");

  bPollEnable = true;
  return 0;
}


void MonomeFtdi::EndpointXtract(uint32_t conf, uint32_t iface, uint32_t alt, uint32_t proto, const USB_ENDPOINT_DESCRIPTOR *pep)
{		
  if (pep->bmAttributes != 2) return;
	
  if (bNumEP == MAX_ENDPOINTS)
    {
      return;
    }

  bConfNum = conf;

  uint32_t index = 0;
  uint32_t pipe = 0;

  if ((pep->bmAttributes & 0x02) == 2)
    {
      index = ((pep->bEndpointAddress & 0x80) == 0x80) ? epDataInIndex : epDataOutIndex;
    }

  // Fill in the endpoint info structure 
  epInfo[index].deviceEpNum = pep->bEndpointAddress & 0x0F;
  epInfo[index].maxPktSize = pep->wMaxPacketSize;

  if (index == epDataInIndex)
    {
      pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum, UOTGHS_HSTPIPCFG_PTYPE_BLK, UOTGHS_HSTPIPCFG_PTOKEN_IN, epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);
    }
  else if (index == epDataOutIndex)
    {
      pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum, UOTGHS_HSTPIPCFG_PTYPE_BLK, UOTGHS_HSTPIPCFG_PTOKEN_OUT, epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);
    }

  // Ensure pipe allocation is okay 
  if (pipe == 0)
    {
      PRINT_DBG("\r\n Pipe allocation failure");
      return;
    }

  epInfo[index].hostPipeNum = pipe;

  bNumEP++;
}


uint32_t MonomeFtdi::Release()
{
  UHD_Pipe_Free(epInfo[epDataInIndex].hostPipeNum);
  UHD_Pipe_Free(epInfo[epDataOutIndex].hostPipeNum);

  // Free allocated USB address 
  pUsb->GetAddressPool().FreeAddress(bAddress);

  // Must have to be reset to 1 
  bNumEP = 1;

  bAddress = 0;

  qNextPollTime = 0;
  bPollEnable = false;

  return 0;
}


// read to internal rx buffer
uint32_t MonomeFtdi::read(void)
{
  return pUsb->inTransfer(bAddress, epInfo[epDataInIndex].deviceEpNum, &rxBytes, rxBuf);
}


// write from external tx buffer
uint32_t MonomeFtdi::write(uint32_t datalen, uint8_t *dataptr)
{
  if (datalen > 255) PRINT_DBG("\r\n WARNING: Trying to send more than 255 bytes down the USB pipe!");

  return pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].deviceEpNum, datalen, dataptr);
}

uint8_t MonomeFtdi::setControlLineState(uint8_t state) 
{
  return ( pUsb->ctrlReq(bAddress, 0, USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE, CDC_SET_CONTROL_LINE_STATE, state, 0, bControlIface, 0, 0, NULL, NULL));
}

uint8_t MonomeFtdi::setLineCoding(const LineCoding *dataptr) 
{
  return ( pUsb->ctrlReq(bAddress, 0, USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE, CDC_SET_LINE_CODING, 0x00, 0x00, bControlIface, sizeof (LineCoding), sizeof (LineCoding), (uint8_t*)dataptr, NULL));
}

uint32_t MonomeFtdi::Poll() {
  uint32_t rcode = 0;
  if(!bPollEnable) { return 0; }
 
  if (qNextPollTime <= millis())
    {
      this->read();
      // keep-alive bytes? or what??
      if(rxBytes > 2) {

	pController->Parse();

	qNextPollTime = millis() + pollPeriod;	
      }
      // need extra 1ms delay after each read... :\
      delay(1);
    }

  return rcode;
}

