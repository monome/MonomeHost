#include <Usb.h>  
#include "MonomeController.h"

USBHost usb;
MonomeController monome(usb);

void GridKeyCallback(byte x, byte y, byte z) { 
  Serial.print("grid key: ");
  Serial.print(x);
  Serial.print(" , ");
  Serial.print(y);
  Serial.print(" , ");
  Serial.print(z);
  Serial.print("\r\n");
  
  monome.led_set(x, y, z > 0 ? 15 : 0 );
  monome.refresh();
}

void ConnectCallback(const char * name, byte cols, byte rows) {
  Serial.print("\r\nmonome device connected; type: ");
  Serial.print(name);
  Serial.print(" ; columns: ");
  Serial.print(cols);
  Serial.print(" ; rows: ");
  Serial.print(rows);
  Serial.print("\r\n");
}


void setup() { 

  // set connection callback
  monome.SetConnectCallback(&ConnectCallback);
  // set key event callback
  monome.SetGridKeyCallback(&GridKeyCallback);

  Serial.begin(19200);
  Serial.print("\n\nStarting...\n");
  delay(200);
}

void loop() { 
  usb.Task();
  /// alternatively, refresh on every tick:
  //  monome.refresh();
}