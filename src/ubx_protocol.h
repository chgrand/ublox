//-*- c++ -*-

#ifndef UBX_PROTOCOL_H
#define UBX_PROTOCOL_H

#define MAX_DATA_LEN 256

#define protoMask_RTCM 0x04
#define protoMask_NMEA 0x02
#define protoMask_UBX  0x01


// SYNC signature
const uint8_t SYNC[2] = {0xB5, 0x62};

typedef struct
{
  uint16_t id;
  uint16_t len;
  uint8_t  data[MAX_DATA_LEN];
} 
ubx_packet_t;

namespace ubx
{
  const static uint16_t CFG_PRT  = 0x0006;
  const static uint16_t CFG_MSG  = 0x0106;
  const static uint16_t CFG_RST  = 0x0406;
  const static uint16_t CFG_RATE = 0x0806;
  const static uint16_t CFG_NMEA = 0x1706;
  const static uint16_t CFG_NAV5 = 0x2406;

  const static uint16_t NAV_POSECEF = 0x0101;
  const static uint16_t NAV_POSLLH  = 0x0201;
  
 
  inline void U4_write(uint8_t* buf, uint32_t value) 
  {
    memcpy(buf, &value, 4);
  }

  inline void U2_write(uint8_t* buf, uint16_t value)
  {
    memcpy(buf, &value, 2);
  }

  inline uint32_t U4_read(uint8_t* buf)
  {
    uint32_t value;
    memcpy(&value, buf,4);
    return value;
  }

  inline int32_t I4_read(uint8_t* buf)
  {
    int32_t value;
    memcpy(&value, buf,4);
    return value;
  }
};


class UBX_Port
{
public:
  UBX_Port() : _connected(false), _debug(false) {};
  ~UBX_Port();


  // serial port configuration and connection
  bool connect(const char *device);
  void disconnect();
  bool isConnected() {return _connected;}
  void set_baudrate(speed_t speed);
  void set_debug(bool flag) {_debug = flag;}
 
  // low-level function read/write packet
  bool read_packet(ubx_packet_t *packet);
  void write_packet(ubx_packet_t *packet);
  bool wait_sync();
  bool wait_sync_timeout(int timeout_ms);
  bool wait_ack();

  // hi-level function
  bool poll_message(ubx_packet_t *packet);
  bool write_message(ubx_packet_t *packet);
  //bool read_message(ubx_packet_t *packet, int timeout=0);  // timeout in 10th of ms

private:
  int _serial_fd;
  bool _connected;
  bool _debug;
};


#endif
