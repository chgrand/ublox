#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <string>

#include <termios.h>
#include "ubx_protocol.h"

UBX_Port ublox;

//-----------------------------------------------------------------------------
bool configure()
{
  if(!ublox.connect("/dev/ttyACM0")) {
    printf("Error cannot connect GPS on port /dev/ttyACM0\n");
    return false;
  }
  ublox.set_baudrate(B115200);

  ubx_packet_t packet;
  
  ublox.set_debug(true);
  
  // set baudrate (Not necessary on usb port)
  uint32_t baudrate=115200;
  uint32_t mode = (3<<6)+(4<<9)+(0<<12);

  packet.id    = ubx::CFG_PRT;
  ublox.poll_message(&packet);
  ubx::U4_write(packet.data+4, mode);
  ubx::U4_write(packet.data+8, baudrate);
  packet.data[14] = protoMask_UBX;      // desactivate NMEA output  
  ublox.write_message(&packet);
  
  // set GNSS freq = 10hz
  packet.id   = ubx::CFG_RATE;
  packet.len  = 6;
  ubx::U2_write(packet.data+0, 100); // Gnss measurement rate in ms
  ubx::U2_write(packet.data+2, 1);   // Navigation Rate, in number of measurement cycles == 1
  ubx::U2_write(packet.data+4, 1);   // Alignment to reference time (0: UTC time, 1: GPS time)
  ublox.write_message(&packet);
  
  // Set GNSS Nav properties
  printf("NAV5-------------\n");
  packet.id    = ubx::CFG_NAV5;
  ublox.poll_message(&packet);
  packet.data[2] = 2;              // dynModel = 2 (stationnary)
  ublox.write_message(&packet);

  //ublox.print(packet);


  // set UBX_MSG active
  packet.id    = ubx::CFG_MSG;
  packet.len  = 3;
  ubx::U2_write(packet.data, ubx::NAV_POSECEF);  // Position in ECEF
  packet.data[2] = 0;                            // At each cycle 
  ublox.write_message(&packet);

  ubx::U2_write(packet.data, ubx::NAV_POSLLH);  // Position in Lat-Long
  packet.data[2] = 1;                            // At each cycle 
  ublox.write_message(&packet);

  ublox.set_debug(false);

  return true;

}


//-----------------------------------------------------------------------------
int process_message()
// return ID of received message
{
  ubx_packet_t packet;

  if(ublox.wait_sync_timeout(10)) { // 10ms timeout
    ublox.read_packet(&packet);
    
    switch(packet.id) {
    case ubx::NAV_POSECEF:
      printf("[%i ms]: NAV_POSECEF = ", ubx::U4_read(packet.data));
      printf("%.3f ", (double)ubx::I4_read(packet.data+4)/100.); 
      printf("%.3f ", (double)ubx::I4_read(packet.data+8)/100.); 
      printf("%.3f ", (double)ubx::I4_read(packet.data+12)/100.); 
      printf("(%.3f)", (double)ubx::I4_read(packet.data+16)/100.); 
      break;

    case ubx::NAV_POSLLH:
      printf("[%i ms]: NAV_POSLLH = ", ubx::U4_read(packet.data));
      printf("%.8f ", (double)ubx::I4_read(packet.data+4)*1e-7); 
      printf("%.8f ", (double)ubx::I4_read(packet.data+8)*1e-7); 
      printf("%.4f ", (double)ubx::I4_read(packet.data+12)/1000.); 
      printf("(%.4f)", (double)ubx::I4_read(packet.data+20)/1000.); 
      break;

    default:
      printf("Unknown id = %0x4x", packet.id);
    }
    printf("\n");
  }
}


//-----------------------------------------------------------------------------
int main(void)
{
  if(!configure())
    return -1;

  while(1)
    process_message();

  return 0;
}
