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

#include "MonomeController.h"

// dummy callbacks
static void ConnectCallbackDummy(const char* str, uint8_t cols, uint8_t rows) 
{ ;; }
static void GridKeyCallbackDummy(uint8_t x, uint8_t y, uint8_t z) 
{ ;; }
static void RingDeltaCallbackDummy(uint8_t n, uint8_t rho)
{ ;; }
static void RingKeyCallbackDummy(uint8_t n, uint8_t z)
{ ;; }

// level above which an LED must be set to be displayed on mono-brightness grid
#define MONOME_VB_CUTOFF 7

// c-tor
MonomeController::MonomeController(USBHost &usb) : 
  parse_serial_(  &MonomeController::parse_serial_dummy ),
  set_intense_(  &MonomeController::set_intense_dummy ),
  grid_led_(  &MonomeController::grid_led_dummy ),
  grid_map_(  &MonomeController::grid_map_dummy ),
  ring_map_(  &MonomeController::ring_map_dummy ),
  refresh_(  &MonomeController::refresh_dummy ),
  frame_dirty_(0),
  ftdi_(usb, this),
  connect_(&ConnectCallbackDummy),
  grid_key_(&GridKeyCallbackDummy),
  ring_delta_(&RingDeltaCallbackDummy),
  ring_key_(&RingKeyCallbackDummy)
{ 
  memset(led_buf_, 0, MONOME_LED_BUF_BYTES);
  memset(tx_buf_, 0, MONOME_TX_BUF_LEN);
}

// d-tor
MonomeController::~MonomeController()
{ 
}


// determine if FTDI string descriptors match monome device pattern
uint8_t MonomeController::CheckDeviceDesc(char* mstr, char* pstr, char* sstr) { 
  char buf[16];
  uint8_t matchMan = 0;
  uint8_t i;
  uint8_t ret;
  uint8_t len;
  //-- source strings are unicode so we need to look at every other byte
  // manufacturer
  for(i=0; i<MONOME_MANSTR_LEN; i++) {
    buf[i] = mstr[i*2];
  }
  buf[i] = 0;

  matchMan = ( strncmp(buf, "monome", MONOME_MANSTR_LEN) == 0 );
  PRINT_DBG("\r\n manstring: ");
  PRINT_DBG(buf);
 
  // serial number string
  for(i=0; i<MONOME_SERSTR_LEN; i++) {
    buf[i] = sstr[i*2];
  }
  buf[i] = 0;

  PRINT_DBG("\r\n serial string: ");
  PRINT_DBG(buf);

  if(matchMan == 0) {
    // didn't match the manufacturer string, but check the serial for DIYs
    if( strncmp(buf, "a40h", 4) == 0) {
      // this is probably an arduinome      
      desc_.protocol = eProtocol40h;
      desc_.device = eDeviceGrid;
      desc_.cols = 8;
      desc_.rows = 8;
      // tilt?
      ret = 1;
    } else {
      // not a monome
      return 0;
    }
  } else { // matched manufctrr string
    if(buf[0] != 'm') {
      // not a monome, somehow. shouldn't happen
      return 0;
    }
    if(buf[3] == 'h') {
      // this is a 40h
      setup_40h(8, 8);
      return 1;
    }
    if( strncmp(buf, "m64-", 4) == 0 ) {
      // series 64
      setup_series(8, 8);
      return 1;
    }
    if( strncmp(buf, "m128-", 5) == 0 ) {
      // series 128
      setup_series(16, 8);
      return 1;
    }
    if( strncmp(buf, "m256-", 5) == 0 ) {
      // series 256
      setup_series(16, 16);
      return 1;
    }
    // if we got here, serial number didn't match series or 40h patterns.
    // so this is probably an extended-protocol device.
    // we need to query for device attributes
    return setup_mext();
  }
  return 0;
}

// set function pointers based on device descriptor
void MonomeController::set_funcs(void)
{
  PRINT_DBG("\r\n MonomeController::set_funcs()");

  if ( desc_.device == eDeviceArc) { 
    refresh_ = &MonomeController::arc_refresh;
  } else {
    refresh_ = &MonomeController::grid_refresh;
  }
  switch(desc_.protocol) { 
  case eProtocol40h :
    parse_serial_ = &MonomeController::parse_serial_40h;
    grid_map_ = &MonomeController::grid_map_40h;
    //      ring_map_ = &MonomeController::ring_map_dummy;
    //      set_intense_ = &MonomeController::set_intense_dummy;
    break;
  case eProtocolSeries: 
    parse_serial_ = &MonomeController::parse_serial_series;
    grid_map_ = &MonomeController::grid_map_series;
    ring_map_ = &MonomeController::ring_map_dummy;
    set_intense_ = &MonomeController::set_intense_series;      
    break;
  case eProtocolMext: 
    parse_serial_ = &MonomeController::parse_serial_mext;
    grid_map_ = &MonomeController::grid_map_mext;
    ring_map_ = &MonomeController::ring_map_mext;
    set_intense_ = &MonomeController::set_intense_mext;      
    break;
  }
}

// refresh from internal buffer
void MonomeController::refresh(void)
{
  (this->*refresh_)(led_buf_);
}

// refresh from external buffer
void MonomeController::refresh(uint8_t* buf)
{
  //  (*refresh)(buf);
  (this->*refresh_)(buf);
}

// check dirty flags and refresh leds
void MonomeController::grid_refresh(uint8_t* buf)
{
  // check quad 0
  if( frame_dirty_ & 0b0001 ) {
    //      PRINT_DBG("\r\n grid_refresh() ; quad 0");
    (this->*grid_map_)(0, 0, buf);
    frame_dirty_ &= 0b1110;
  }
  // check quad 1
  if( frame_dirty_ & 0b0010 ) {
    //      PRINT_DBG("\r\n grid_refresh() ; quad 1");
    if ( desc_.cols > 7 ) {
      (this->*grid_map_)(8, 0, buf + 8);
      frame_dirty_ &= 0b1101;
    }
  }
  // check quad 2
  if( frame_dirty_ & 0b0100 ) { 
    //      PRINT_DBG("\r\n grid_refresh() ; quad 2");
    if( desc_.rows > 7 ) {
      (this->*grid_map_)(0, 8, buf + 128);
      frame_dirty_ &= 0b1011;
    }
  }
  // check quad 3
  if( frame_dirty_ & 0b1000 ) {
    //      PRINT_DBG("\r\n grid_refresh() ; quad 3");
    if( (desc_.rows > 7) && (desc_.cols > 7) )  {
      (this->*grid_map_)(8, 8, buf + 136);
      frame_dirty_ &= 0b0111;
    }
  }
}


// check flags and refresh arc
void MonomeController::arc_refresh(uint8_t* buf)
{
  // may need to wait after each quad until tx transfer is complete
  uint8_t i;

  for(i=0; i<desc_.encs; i++) {
    if(frame_dirty_ & (1<<i)) {
      (this->*ring_map_)(i, buf + (i<<6));
      frame_dirty_ &= ~(1<<i);
    }
  }
}


// set quadrant refresh flag from pos
void MonomeController::calc_quadrant_flag(uint8_t x, uint8_t y) {
  if(x > 7) {
    if (y > 7) {      
      frame_dirty_ |= 0b1000;
    }
    else {
      frame_dirty_ |= 0b0010;
    }
  } else {
    if (y > 7) {
      frame_dirty_ |= 0b0100;
    }
    else {
      frame_dirty_ |= 0b0001;
    }
  } 
}

// set given quadrant dirty flag
void MonomeController::set_quadrant_flag(uint8_t q) {
  frame_dirty_ |= (1 << q);
}


// convert flat framebuffer idx to x,y
void MonomeController::idx_xy(uint32_t idx, uint8_t* x, uint8_t* y) {
  *x = idx & 0xf;
  *y = (idx >> 4);
}

// convert x,y to framebuffer idx
uint32_t MonomeController::xy_idx(uint8_t x, uint8_t y) {
  return x | (y << 4);
}

// grid led/set function
void MonomeController::grid_led_set(uint8_t x, uint8_t y, uint8_t z) {
  led_buf_[MonomeController::xy_idx(x, y)] = z;
  MonomeController::calc_quadrant_flag(x, y);
}


// grid led/clear function
void MonomeController::grid_led_clear() {
  for(int i = 0;i<256;i++)
    led_buf_[i] = 0;
  frame_dirty_ = 0xf;
}

// grid led/toggle function
void MonomeController::grid_led_toggle(uint8_t x, uint8_t y) {
  led_buf_[xy_idx(x,y)] ^= 0xff;
  calc_quadrant_flag(x, y);  
}

// arc led/set function
///// FIXME??? totally untested
void MonomeController::arc_led_set(uint8_t enc, uint8_t ring, uint8_t val) {
  led_buf_[ring + (enc << 6)] = val;
  frame_dirty_ |= (1 << enc);
}

  
/////////////////////////
//// protocol-specific

// dummies
void MonomeController::parse_serial_dummy (void) { ;; }
void MonomeController::set_intense_dummy (uint8_t level) { ;; }
void MonomeController::grid_led_dummy (uint8_t x, uint8_t y, uint8_t val) { ;; }
void MonomeController::grid_map_dummy (uint8_t x, uint8_t y, uint8_t* data) { ;; }
void MonomeController::ring_set_dummy (uint8_t n, uint8_t rho, uint8_t val) { ;; }
void MonomeController::ring_map_dummy (uint8_t n, uint8_t* data) { ;; }
void MonomeController::refresh_dummy (uint8_t* buf) { ;; }

//-------------
//---- setup

// setup 40h-protocol device
void MonomeController::setup_40h(uint8_t cols, uint8_t rows)
{
  desc_.protocol = eProtocol40h;
  desc_.device = eDeviceGrid;
  desc_.cols = 8;
  desc_.rows = 8;
  desc_.vari = 0;
  set_funcs();
  (*connect_)("40h", cols, rows);
}

// setup series device
void MonomeController::setup_series(uint8_t cols, uint8_t rows)
{
  PRINT_DBG("\r\n setup series");
  desc_.protocol = eProtocolSeries;
  desc_.device = eDeviceGrid;
  desc_.cols = cols;
  desc_.rows = rows;
  desc_.vari = 0;
  desc_.tilt = 1;
  set_funcs();
  (*connect_)("series", cols, rows);
}

// setup extended device, return success /failure of query
uint8_t MonomeController::setup_mext(void)
{
  uint8_t* prx;
  uint8_t w = 0;
  uint8_t rx_bytes;

  PRINT_DBG("\r\n setup mext");

  desc_.protocol = eProtocolMext;

  desc_.vari = 1;

  rx_bytes = 0;

  while(rx_bytes != 6) {
    delay(1);
    ftdi_.write(1, &w);  // query  

    delay(1);
    ftdi_.read();

    delay(1);
    rx_bytes = ftdi_.rx_bytes();
  }
  
  prx = ftdi_.rx_buf();

  prx++; // 1st returned byte is 0
  if(*prx == 1) {
    desc_.device = eDeviceGrid;
    prx++;
    if(*prx == 1) {
      PRINT_DBG("\r\n monome 64");
      desc_.rows = 8;
      desc_.cols = 8;
    }
    else if(*prx == 2) {
      PRINT_DBG("\r\n monome 128");
      desc_.rows = 8;
      desc_.cols = 16;
    }
    else if(*prx == 4) {
      PRINT_DBG("\r\n monome 256");
      desc_.rows = 16; 
      desc_.cols = 16;
    }
    else {
      return 0; // bail
    }   
    desc_.tilt = 1;
  }
  else if(*prx == 5) {
    desc_.device = eDeviceArc;
    desc_.encs = *(++prx);
  } else {
    return 0; // bail
  }

  // get id
  w = 1;
  delay(1);
  ftdi_.write(1, &w);
  delay(1);
  ftdi_.read();
  delay(1);

  rx_bytes = ftdi_.rx_bytes();
  prx = ftdi_.rx_buf();
  if(*(prx+2) == 'k')
    desc_.vari = 0;
  set_funcs();
  (*connect_)("mext", desc_.cols, desc_.rows);

  return 1;
}

//------------------
//--- rx for each protocol

/// parse serial input from device
/// should be called when read is complete
/// (e.g. from usb transfer callback )

void MonomeController::parse_serial_40h(void)
{
  uint8_t* prx = ftdi_.rx_buf();
  uint8_t i;
  uint8_t rx_bytes = ftdi_.rx_bytes();

  i = 0;
  while(i < rx_bytes) {
    // press event
    if ((prx[0] & 0xf0) == 0) {

      (*grid_key_)(
	      ((prx[1] & 0xf0) >> 4),
	      prx[1] & 0xf,
	      ((prx[0] & 0xf) != 0)
	      );
    }
    
    i += 2;
    prx += 2;
  }
}

void MonomeController::parse_serial_series(void)
{
  uint8_t* prx = ftdi_.rx_buf();
  uint8_t x, y, z;
  // FTDI reports 2 status bytes each read.
  // driver class is responsible for stripping these
  uint8_t rx_bytes = ftdi_.rx_bytes();
  uint8_t i = 0;

  while(i < rx_bytes) {
    // process consecutive pairs of bytes
    (*grid_key_)( 
	    ((prx[1] & 0xf0) >> 4) ,
	    prx[1] & 0xf,
	    ((prx[0] & 0xf0) == 0)
	     );
    i += 2;
    prx += 2;
  }

}

void MonomeController::parse_serial_mext(void) {
  uint8_t nbp; // number of bytes processed
  uint8_t* prx; // pointer to rx buf
  uint8_t com;
  uint8_t rx_bytes;

  rx_bytes = ftdi_.rx_bytes();
  if( rx_bytes ) {
    nbp = 0;
    prx = ftdi_.rx_buf();
    while(nbp < rx_bytes) {
      com = (uint8_t)(*(prx++));
      nbp++;
      switch(com) {
      case 0x20: // grid key up
	(*grid_key_)( *prx, *(prx+1), 0);
	nbp += 2;
	prx += 2;
	break;
      case 0x21: // grid key down
	(*grid_key_)( *prx, *(prx+1), 1);
	nbp += 2;
	prx += 2;
	break;
      case 0x50: // ring delta
	(*ring_delta_)( *prx, *(prx+1));
	nbp += 2;
	prx += 2;
	break;
      case 0x51 : // ring key up
	(*ring_key_)( *prx++, 0 );
	prx++;
	break;
      case 0x52 : // ring key down
	(*ring_key_)( *prx++, 1 );
	nbp++;
	break;
	/// TODO: more commands... 
      default:
	return;
      }
    } 
  }
}

//-----------------------------------------
//--- tx

// update a whole frame
// note that our input data is one byte per led!!
// NOTE (FIXME?): map() always performs varibright update... 
void MonomeController::grid_map_mext( uint8_t x, uint8_t y, uint8_t* data )
{
  uint8_t* ptx;
  uint8_t i, j;

  tx_buf_[0] = 0x1A;  
  tx_buf_[1] = x;
  tx_buf_[2] = y;
  
  ptx = tx_buf_ + 3;
  
  // copy and convert
  for(i=0; i<MONOME_QUAD_LEDS; i++) {
    for(j=0; j<4; j++) {
      // binary value of data byte to bitfield of tx byte
      *ptx = (*data) << 4;
      data++;
      *ptx |= *data;
      data++;
      ptx++;
    }
    data += MONOME_QUAD_LEDS; // skip the rest of the row to get back in target quad
  }
  ftdi_.write(32 + 3, tx_buf_);
}


void MonomeController::grid_map_40h(uint8_t x, uint8_t y, uint8_t* data)
{
  //  static uint8_t i, j;
  uint8_t i, j;
  // ignore all but first quadrant -- do any devices larger than 8x8 speak 40h?
  if (x != 0 || y != 0) {
    return;
  }
  for(i=0; i<MONOME_QUAD_LEDS; i++) {
    // led row command + row number
    tx_buf_[(i*2)] = 0x70 + i;
    tx_buf_[(i*2)+1] = 0;
    PRINT_DBG("\r\n * data bytes: ");
    for(j=0; j<MONOME_QUAD_LEDS; j++) {
      // set row bit if led should be on
      tx_buf_[(i*2)+1] |= ((*data > 0) << j);
      // advance data to next bit
      ++data;
    }
    // skip next 8 bytes to get to next row
    data += MONOME_QUAD_LEDS;
  }
  ftdi_.write(16, tx_buf_);
}

void MonomeController::grid_map_series(uint8_t x, uint8_t y, uint8_t* data)
{
  uint8_t * ptx;
  uint8_t i, j;

  //    PRINT_DBG("\r\n grid_map_series()");
    
  // command (upper nibble)
  tx_buf_[0] = 0x80;
  // quadrant index (lower nibble, 0-3)
  tx_buf_[0] |= ( (x > 7) | ((y > 7) << 1) );

  // pointer to tx data
  ptx = tx_buf_ + 1;
  
  // copy and convert
  for(i=0; i<MONOME_QUAD_LEDS; i++) {
    *ptx = 0;
    for(j=0; j<MONOME_QUAD_LEDS; j++) {
      // binary value of data byte to bitfield of tx byte
      *ptx |= ((*data > MONOME_VB_CUTOFF) << j);
      ++data;
    }
    // skip the rest of the row to get back in target quad
    data += MONOME_QUAD_LEDS; 
    ++ptx;
  }
  ftdi_.write(MONOME_QUAD_LEDS + 1, tx_buf_);  
}

void MonomeController::ring_map_mext(uint8_t n, uint8_t* data)
{
  uint8_t* ptx;
  uint8_t i;

  tx_buf_[0] = 0x92;
  tx_buf_[1] = n;
  
  ptx = tx_buf_ + 2;
  
  // smash 64 LEDs together, nibbles
  for(i=0; i<32; i++) {
    *ptx = *data << 4;
    data++;
    *ptx |= *data;
    data++;
    ptx++;
  }

  ftdi_.write(32 + 2, tx_buf_);
}

void MonomeController::set_intense_series(uint8_t v)
{
  // message id: (10) intensity
  // bytes:    1
  // format:   iiiibbbb
  // i (message id) = 10
  // b (brightness) = 0-15 (4 bits)
  // encode:   byte 0 = ((id) << 4) | b = 160 + b

  tx_buf_[0] = 0xa0;
  tx_buf_[0] |= (v & 0x0f);
  ftdi_.write(1, tx_buf_);
}

void MonomeController::set_intense_mext(uint8_t v)
{
  // TODO...
}
