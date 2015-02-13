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


#ifndef _MONOME_HOST__MONOME_CONTROLLER_H
#define _MONOME_HOST__MONOME_CONTROLLER_H

#include "MonomeFTDI.h"

// callback types
// rather foolishly declared outside any class/namespace...
typedef void(*MonomeConnectCallback)(const char* str, uint8_t cols, uint8_t rows);
typedef void(*MonomeGridKeyCallback)(uint8_t x, uint8_t y, uint8_t z);
typedef void(*MonomeRingDeltaCallback)(uint8_t n, uint8_t rho);
typedef void(*MonomeRingKeyCallback)(uint8_t n, uint8_t z);

class MonomeController : public MonomeReportParser { 
 public:

  //---- device enumeration
  typedef enum {
    eDeviceGrid,   /// any grid device
    eDeviceArc,     /// any arc device
    eDeviceNumDevices // dummy and count
  } device_t;

  // protocol enumeration
  typedef enum {
    eProtocol40h,      /// 40h and arduinome protocol (pre-2007)
    eProtocolSeries,   /// series protocol (2007-2011)
    eProtocolMext,     /// extended protocol (2011 - ? ), arcs + grids  
    eProtocolNumProtocols // dummy and count
  } protocol_t;

  // device descriptor
  typedef struct desc {
    protocol_t protocol;
    device_t device;
    uint8_t cols;  	// number of columns
    uint8_t rows;  	// number of rows
    uint8_t encs; 	// number of encoders
    uint8_t tilt;  	// has tilt
    uint8_t vari; 	// is variable brightness, true/false
  } desc_t;

  // c-tor / d-tor
  MonomeController(USBHost& usb);
  ~MonomeController();

  // initialize
  void init(void);

  // parse serial data
  //  void Parse(MonomeFtdi* ftdi, uint32_t len, uint8_t *buf) {
  void Parse(void) {
    /// FIXME
    /// as noted in MonomeFTDI.h, this is pretty bad form
    (this->*parse_serial_)();
  }

  // check monome device from FTDI string descriptors
  uint8_t CheckDeviceDesc(char* mstr, char* pstr, char* sstr);

  // refresh from internal LED buffer
  void refresh(void);
  // refresh from arbitrary LED buffer
  void refresh(uint8_t* buf);
  // check dirty flags and refresh leds
  void grid_refresh(uint8_t* buf);
  // check dirty flags and refresh leds
  void arc_refresh(uint8_t* buf);

  //------------------------
  //---- manipulate internal LED buffer

  //-- convenience...
  void led_set(uint8_t x, uint8_t y, uint8_t val) {
    if(desc_.device == eDeviceArc) {
      arc_led_set(x, y, val);
    } else {
      grid_led_set(x, y, val);
    }
  }

  // grid led/set function
  void grid_led_set(uint8_t x, uint8_t y, uint8_t val);
  // grid led/toggle function
  void grid_led_toggle(uint8_t x, uint8_t y);
  // arc led/set function
  void arc_led_set(uint8_t enc, uint8_t ring, uint8_t val);
  // set quadrant dirty flag from (x,y)
  void calc_quadrant_flag(uint8_t x, uint8_t y);
  // set given quadrant dirty flag
  void set_quadrant_flag(uint8_t q);
  // convert flat framebuffer idx to x,y
  void idx_xy(uint32_t idx, uint8_t* x, uint8_t* y);
  // convert x,y to framebuffer idx
  uint32_t xy_idx(uint8_t x, uint8_t y);

  uint8_t size_x(void) { return desc_.cols; }
  uint8_t size_y(void) { return desc_.rows; }
  uint8_t is_vari(void) { return desc_.rows; }

  device_t device_type(void) { return desc_.device; }


 private:
  //----- function prototypes for device-agnostic commands
  /// parse raw serial data (all devices)
  typedef void(MonomeController::*parse_serial_t)(void);
  // set led intensity of connected device
  typedef void(MonomeController::*set_intense_t)(uint8_t level);

  //-- write (grid)
  // write single led
  typedef void(MonomeController::*grid_led_t)(uint8_t x, uint8_t y, uint8_t val);
  // write binary 8x8 frame
  typedef void(MonomeController::*grid_map_t)(uint8_t x, uint8_t y, uint8_t* data);

  //-- write (ring)
  // set single led in ring with 4b value
  typedef void(MonomeController::*ring_set_t)(uint8_t n, uint8_t rho, uint8_t val);
  //  set all leds in ring 
  typedef void(MonomeController::*ring_map_t)(uint8_t n, uint8_t* data);

  // refresh LEDs
  typedef void(MonomeController::*refresh_t)(uint8_t* buf);

  /// dummies
  void parse_serial_dummy (void) ; 
  void set_intense_dummy (uint8_t level) ; 
  void grid_led_dummy (uint8_t x, uint8_t y, uint8_t val) ; 
  void grid_map_dummy (uint8_t x, uint8_t y, uint8_t* data) ; 
  void ring_set_dummy (uint8_t n, uint8_t rho, uint8_t val) ;
  void ring_map_dummy (uint8_t n, uint8_t* data) ;
  void refresh_dummy (uint8_t* buf) ; 

  // setup for each protocol
  void setup_40h(uint8_t cols, uint8_t rows);
  void setup_series(uint8_t cols, uint8_t rows);
  uint8_t setup_mext(void);

  // rx for each protocol
  void parse_serial_40h(void);
  void parse_serial_series(void);
  void parse_serial_mext(void);

  // set intensity
  void set_intense_series(uint8_t level);
  void set_intense_mext(uint8_t level);

  // tx for each protocol
  void grid_map_40h(uint8_t x, uint8_t y, uint8_t* data);
  void grid_map_series(uint8_t x, uint8_t y, uint8_t* data);
  void grid_map_mext(uint8_t x, uint8_t y, uint8_t* data);

  void ring_map_series(uint8_t n, uint8_t* data);
  void ring_map_mext(uint8_t n, uint8_t* data);

  // set function pointers based on device descriptor
  void set_funcs(void);

  // FTDI driver
  MonomeFtdi ftdi_;
    
  // device descriptor
  desc_t desc_;

  //--- pointers to device-specific functions
  parse_serial_t 	parse_serial_;
  set_intense_t 	set_intense_;
  grid_led_t 		grid_led_;
  grid_map_t 		grid_map_;
  ring_map_t 		ring_map_;
  refresh_t 		refresh_;

  //--- internal LED state
  // dirty flags for each quadrant or knob (bitwise)
  uint8_t frame_dirty_;
  // a buffer big enough to hold all led data for 256 or arc4
  // each led gets a full byte
  uint8_t led_buf_[MONOME_LED_BUF_BYTES];

  // local tx buffer
  uint8_t  tx_buf_[MONOME_TX_BUF_LEN];

  // callback pointers
  MonomeConnectCallback connect_;
  MonomeGridKeyCallback grid_key_;
  MonomeRingDeltaCallback ring_delta_;
  MonomeRingKeyCallback ring_key_;

 public:

  void SetConnectCallback(MonomeConnectCallback fn) {
    connect_ = fn;
  }

  void SetGridKeyCallback(MonomeGridKeyCallback fn) {
    grid_key_ = fn;
  }

  void SetRingDeltaCallback(MonomeRingDeltaCallback fn) { 
    ring_delta_ = fn;
  }

  void SetRingKeyCallback(MonomeRingKeyCallback fn) { 
    ring_key_ = fn;
  }

};

#endif
