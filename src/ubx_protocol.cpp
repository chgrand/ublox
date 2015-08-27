#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <stdint.h>  // Standard int typedef
#include <time.h>    // POSIX time
#include <inttypes.h>
#include "ubx_protocol.h"


//-----------------------------------------------------------------------------
// Utility function to get timestamp
static int get_time_us()
{
    struct timespec tms;

    // POSIX.1-2008
    if (clock_gettime(CLOCK_REALTIME,&tms))
        return -1;

    // seconds, multiplied with 1e6
    int64_t u_sec = tms.tv_sec * 1e6;
    // Add full microseconds
    u_sec += tms.tv_nsec/1000;
    // round up if necessary
    if (tms.tv_nsec % 1000 >= 500)
        ++u_sec;

    return u_sec;
}


//-----------------------------------------------------------------------------
UBX_Port::~UBX_Port()
{
    if(_connected)
        close(_serial_fd);
}

//-----------------------------------------------------------------------------
bool UBX_Port::connect(const char *device)
{
    // Doc: http://mirror.datenwolf.net/serial/

    _connected = false;

    int fd = open(device, O_RDWR | O_NOCTTY);// | O_NDELAY);
    if(fd==-1)
        return false;

    struct termios options;
    tcgetattr(fd, &options);

    // Set the baud rates to 9600
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);

    // No parity (8N1)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Disable hardware flow control:
    options.c_cflag &= ~CRTSCTS;

    // Disable software flow control:
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Configure line as raw input:
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Configure line as raw output:
    options.c_oflag &= ~OPOST;

    // Timeout = 10ms
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;

    // Set the new options for the port:
    tcsetattr(fd, TCSANOW, &options);

    _serial_fd = fd;
    _connected = true;
    return true;
}

//-----------------------------------------------------------------------------
void UBX_Port::disconnect()
{
    if(_connected) {
        close(_serial_fd);
        _connected = false;
    }
}

//-----------------------------------------------------------------------------
void UBX_Port::set_baudrate(speed_t speed)
{
    struct termios options;
    tcgetattr(_serial_fd, &options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    tcsetattr(_serial_fd, TCSANOW, &options);
}



//-----------------------------------------------------------------------------
void UBX_Port::print_packet(const ubx_packet_t *packet)
{
    //printf("%04x ", packet->id);
    printf("<%02x, %02x> ", (packet->id)&0xFF, (packet->id)>>8);
    printf("(% 3i): ", packet->len);
    for(int i=0; i<packet->len; i++)
        printf("%02x ", packet->data[i]);
    printf("\n");
}

//-----------------------------------------------------------------------------
bool UBX_Port::read_packet(ubx_packet_t *packet)
{
    uint8_t cka=0,ckb=0;
    uint8_t buffer[MAX_DATA_LEN];
    int n;

    n = read(_serial_fd, buffer, 4);
    if(n!=4)
        return false;

    int data_len = buffer[2]+(buffer[3]<<8);
    if(data_len>MAX_DATA_LEN) {
        printf("UBX_READ error: invalid data lenght <");
        for(int i=0; i<4; i++)
            printf("%02X ", buffer[i]);
        printf(">\n");
        return false;
    }

    n = read(_serial_fd, (buffer+4), data_len+2);
    if(n!=data_len+2)
        return false;

    // compute checksum
    for (int i=0; i < data_len+4; i++) {
        cka+=buffer[i];
        ckb+=cka;
    }

    if ((cka!=buffer[data_len+4])||(ckb!=buffer[data_len+5])) {
        printf("UBX_READ checksum error\n");
        return false;
    }

    memcpy(packet, buffer, data_len+6);

    if(_debug) {
        printf("ubx_read : ");
        print_packet(packet);
    }

    return true;
}


//-----------------------------------------------------------------------------
void UBX_Port::write_packet(const ubx_packet_t *packet)
{
    uint16_t data_len = packet->len;
    uint8_t buffer[MAX_DATA_LEN];
    uint8_t cka=0;
    uint8_t ckb=0;

    buffer[0] = SYNC[0];
    buffer[1] = SYNC[1];
    memcpy(buffer+2, packet, data_len+4);

    for(int i=0; i<data_len+4; i++) {
        cka+=buffer[i+2];
        ckb+=cka;
    }

    buffer[data_len+6] = cka;
    buffer[data_len+7] = ckb;

    if(_debug) {
        printf("ubx_write: ");
        print_packet(packet);
    }

    write(_serial_fd, buffer, data_len+8);
}


//-----------------------------------------------------------------------------
bool UBX_Port::wait_sync()
{
    uint8_t byte;
    if(read(_serial_fd, &byte, 1)!=1) return false;
    if(byte!=SYNC[0]) return false;
    if(read(_serial_fd, &byte, 1)!=1) return false;
    if(byte!=SYNC[1]) return false;
    return true;
}

//-----------------------------------------------------------------------------
bool UBX_Port::wait_sync_timeout(int timeout_ms)
{
    int time_0 = get_time_us();

    do {
        if(wait_sync()) return true;
    }
    while( (get_time_us()-time_0)<timeout_ms*1000);
    if(_debug)
        printf("timeout = %i\n", get_time_us()-time_0);
    return false;
}

//-----------------------------------------------------------------------------
bool UBX_Port::wait_ack()
{
    ubx_packet_t packet;
    if(!wait_sync_timeout(10)) return false;
    if(!read_packet(&packet)) return false;
    if( packet.id != 0x0105)  return false;
    return true;
}


//-----------------------------------------------------------------------------
bool UBX_Port::poll_message(ubx_packet_t *packet)
{
    packet->len=0;
    write_packet(packet);
    if(!wait_sync_timeout(1000)) return false;
    read_packet(packet);
    return wait_ack();
}

//-----------------------------------------------------------------------------
bool UBX_Port::write_message(ubx_packet_t *packet)
{
    write_packet(packet);
    return wait_ack();
}

