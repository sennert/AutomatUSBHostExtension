/* 
   AUTOMAT USB HOST EXTENSION FIRMWARE
   ===================================

   MIDI USB HOST to MIDI over I2C converter for SAMD21 and SAMD51 Arduino and
   Arduino compatible boards.
   Can be used as Firmware for Dadamachines Automat USB Host Extension

   USB Host Controller receives incoming MIDI Messages from a MIDI Controller 
   conntected to the USB Host Port, filters NoteOn & NoteOff Messages
   and sends them over to the Automat Controller via I2C Bus using the
   2x6 Pin Connector of the Side of Automat Controller

   Copyright (c) 2019, DADAMACHINES
   Author: Steffen Sennert
   All rights reserved.

   The Software uses the USB HOST Library SAMD by gsports (gdsports625@gmail.com)
   https://github.com/gdsports/USB_Host_Library_SAMD

   Which is based on the USB Host Shield 2.0 Library by felis
   https://github.com/felis/USB_Host_Shield_2.0 (do NOT use this Library! download the link above)
   
   
   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:
   
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
    3. Neither the name of DADAMACHINES nor the names of its contributors may be used
       to endorse or promote products derived from this software without
       specific prior written permission.
       
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <Wire.h>
#include <MIDI.h>
#include <usbh_midi.h>
#include <usbhub.h>

#define I2C_ADDRESS_RECEIVER 0x60    // Dadamachines Automat I2C Address


//------------------------------------------------------------------------------------------------------------------------------------------
// Satisfy the IDE, which needs to see the include statment in the ino too.
//#ifdef dobogusinclude
//#include <spi4teensy3.h>
//#include <SPI.h>
//#endif
//#ifdef MIDI_CREATE_DEFAULT_INSTANCE
//MIDI_CREATE_DEFAULT_INSTANCE();
//#endif
#ifdef USBCON
#define _MIDI_SERIAL_PORT Serial1
#else
#define _MIDI_SERIAL_PORT Serial
#endif
//--------------------------------------------------------------------------------------------------------------------------------------------


USBHost UsbH;
//USBHub Hub(&UsbH);
USBH_MIDI  MIDIUSBH(&UsbH);


unsigned long t1 = micros();
unsigned long last_RX = 0;


void setup()
{
  Wire.setClock(100000);
  Wire.begin(); // join i2c bus (address optional for master)
  SERIAL_PORT_MONITOR.begin(115200);

  if (UsbH.Init() == -1) {
//    SERIAL_PORT_MONITORL.println(F("USB host failed to start"));
    while (1); //halt
  }
  
  delay(100);
}


void loop()
{
  t1 = micros();

  if (t1 - last_RX >= 16383) {
    midiTask();
  }
}


void midiTask() {
  UsbH.Task();

  if ( UsbH.getUsbTaskState() == USB_STATE_RUNNING )
  {
    USBHost_to_I2C();
    last_RX = micros();
  }
}


void sendMIDIoverI2C(byte _byte1, byte _byte2, byte _byte3)
{
  Wire.beginTransmission(I2C_ADDRESS_RECEIVER); // transmit to device (0x60)
  Wire.write(0x01);                  // infoByte, lets reeceiver know that it receives a MIDI_I2C_Event Message, not direct Pin+Velo Set  
  Wire.write(_byte1);             // sends Channel and EventType byte  
  Wire.write(_byte2);             // sends Note byte  
  Wire.write(_byte3);             // sends Velocity byte  
//  Wire.write(0);                  // Not sure if necessary  
  Wire.endTransmission();         // stop transmitting
}


void USBHost_to_I2C()
{
  uint8_t recvBuf[MIDI_EVENT_PACKET_SIZE];
  uint8_t rcode = 0;     //return code
  uint16_t rcvd;
  uint8_t readCount = 0;

  rcode = MIDIUSBH.RecvData( &rcvd, recvBuf);

  //data check
  if (rcode != 0 || rcvd == 0) return;
  if ( recvBuf[0] == 0 && recvBuf[1] == 0 && recvBuf[2] == 0 && recvBuf[3] == 0 ) {
    return;
  }

  uint8_t *p = recvBuf;
  while (readCount < rcvd)  {
    
    if (*p == 0 && *(p + 1) == 0) break; //data end
//    SERIAL_PORT_MONITOR.print(F("Incoming USB MIDI: "));
//    SERIAL_PORT_MONITOR.print(p[0], DEC);
//    SERIAL_PORT_MONITOR.print(' ');
//    SERIAL_PORT_MONITOR.print(p[1], DEC);
//    SERIAL_PORT_MONITOR.print(' ');
//    SERIAL_PORT_MONITOR.print(p[2], DEC);
//    SERIAL_PORT_MONITOR.print(' ');
//    SERIAL_PORT_MONITOR.println(p[3], DEC);
    uint8_t header = *p & 0x0F;
    p++;
    switch (header) {
//      case 0x00:  // Misc. Reserved for future extensions.
//      case 0x01:  // Cable events. Reserved for future expansion.
//      case 0x02:  // Two-byte System Common messages
//      case 0x0C:  // Program Change
//      case 0x0D:  // Channel Pressure
//      case 0x03:  // Three-byte System Common messages
        case 0x08:  // Note-off
          sendMIDIoverI2C(p[0],p[1],p[2]);
          break;
        case 0x09:  // Note-on
          sendMIDIoverI2C(p[0],p[1],p[2]);
          break;
//      case 0x0A:  // Poly-KeyPress
//      case 0x0B:  // Control Change
//      case 0x0E:  // PitchBend Change
//      case 0x04:  // SysEx starts or continues
//      case 0x05:  // Single-byte System Common Message or SysEx ends with the following single byte
//      case 0x06:  // SysEx ends with the following two bytes
//      case 0x07:  // SysEx ends with the following three bytes
//      case 0x0F:  // Single Byte, TuneRequest, Clock, Start, Continue, Stop, etc.
        default:
          break;
    }
    p += 3;
    readCount += 4;
  }
}
