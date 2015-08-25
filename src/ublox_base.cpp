#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <string>

#include <termios.h>
#include "ubx_protocol.h"
#include "gps_time.h"

namespace STATE
{
    const static uint8_t INIT = 0x01;
    const static uint8_t RUN  = 0x02;
}
uint8_t state;



const static uint8_t NO_FIX = 0x00;
const static uint8_t FIX_DR = 0x01;
const static uint8_t FIX_2D = 0x02;
const static uint8_t FIX_3D = 0x03;
const static uint8_t FIX_DR_GPS = 0x04;
const static uint8_t FIX_TIME = 0x05;

const static char *fix_message[] =
{
    "No fix",
    "Dead Reckoning only",
    "2D-fix",
    "3D-fix",
    "GPS + dead reckoning combined",
    " Time only fix"
};

// NAV-SOL
typedef struct
{
    gps_time_t time;
    int fix_type;
    bool dgps_fix;
    double position[3];
    double position_accuracy;
    double velocity[3];
    double velocity_accuracy;
    double p_dop;
    int num_SV;
} ecef_solution_t;

// NAV-STATUS
typedef struct
{
    int fix_type;
    bool dgps_fix;
    bool pr_prr_correction;
    enum power_mode {ACQUISITION, TRACKING, POWER_OPTIMIZED_TRACKING, INACTIVE} power_state;
} status_t;





ecef_solution_t ecef_solution;
double ecef_mean_position[3];

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
    ubx::U2_write(packet.data+2, 1);   // Nav. Rate, in number of meas. cycles
    ubx::U2_write(packet.data+4, 1);   // Ref. time (0: UTC time, 1: GPS time)
    ublox.write_message(&packet);

    // Set GNSS Nav properties
    printf("NAV5-------------\n");
    packet.id    = ubx::CFG_NAV5;
    ublox.poll_message(&packet);
    packet.data[2] = 2;              // dynModel = 2 (stationnary)
    ublox.write_message(&packet);

    // Select active UBX_MSG
    packet.id    = ubx::CFG_MSG;
    packet.len  = 3;
    ubx::U2_write(packet.data, ubx::NAV_SOL);  // Position in ECEF
    packet.data[2] = 1;                            // At each cycle
    ublox.write_message(&packet);

    ubx::U2_write(packet.data, ubx::NAV_SAT);  // Position in Lat-Long
    packet.data[2] = 1;                            // At each cycle
    ublox.write_message(&packet);

    ublox.set_debug(false);

    return true;

}

//-----------------------------------------------------------------------------
int read_data()
{
    ubx_packet_t packet;
    double iTow;
    double fTow;
    int week;

    if(!ublox.wait_sync_timeout(10)) // 10ms timeout
        return -1;

    if(!ublox.read_packet(&packet))
        return -1;

    switch(packet.id) {
    case ubx::NAV_POSECEF:
        printf("[%i ms]: NAV_POSECEF = ", ubx::U4_read(packet.data));
        printf("%.3f ", (double)ubx::I4_read(packet.data+4)/100.);
        printf("%.3f ", (double)ubx::I4_read(packet.data+8)/100.);
        printf("%.3f ", (double)ubx::I4_read(packet.data+12)/100.);
        printf("(%.3f)\n", (double)ubx::I4_read(packet.data+16)/100.);
        break;

    case ubx::NAV_POSLLH:
        printf("[%i ms]: NAV_POSLLH = ", ubx::U4_read(packet.data));
        printf("%.8f ", (double)ubx::I4_read(packet.data+4)*1e-7);
        printf("%.8f ", (double)ubx::I4_read(packet.data+8)*1e-7);
        printf("%.4f ", (double)ubx::I4_read(packet.data+12)/1000.);
        printf("(%.4f)\n", (double)ubx::I4_read(packet.data+20)/1000.);
        break;

    case ubx::NAV_SOL:
        iTow = (double)ubx::U4_read(packet.data+0)*1e-3;
        fTow = (double)ubx::I4_read(packet.data+4)*1e-9;
        week = int(ubx::I2_read(packet.data+8));

        //ecef_solution.time = iTow+fTow+week*(24*7*3600);
        gps2epoch(ecef_solution.time, week, iTow+fTow);
        ecef_solution.fix_type = (int)ubx::U1_read(packet.data+10);
        ecef_solution.dgps_fix = (bool)(ubx::U1_read(packet.data+11)&0x04);
        ecef_solution.position[0] = (double)ubx::I4_read(packet.data+12)/100.;
        ecef_solution.position[1] = (double)ubx::I4_read(packet.data+16)/100.;
        ecef_solution.position[2] = (double)ubx::I4_read(packet.data+20)/100.;
        ecef_solution.position_accuracy = (double)ubx::U4_read(packet.data+24)/100.;
        ecef_solution.velocity[0] = (double)ubx::I4_read(packet.data+28)/100.;
        ecef_solution.velocity[1] = (double)ubx::I4_read(packet.data+32)/100.;
        ecef_solution.velocity[2] = (double)ubx::I4_read(packet.data+36)/100.;
        ecef_solution.velocity_accuracy = (double)ubx::U4_read(packet.data+40)/100.;
        ecef_solution.p_dop = (double)ubx::U2_read(packet.data+44)*0.01;
        ecef_solution.num_SV = (int)ubx::U1_read(packet.data+47);
        break;

    default:
        printf("Unknown id = %0x4x\n", packet.id);
        return -1;
    }
    return packet.id;
}


//-----------------------------------------------------------------------------
bool update()
{
    uint16_t message_id;

    switch(state)
    {
    case STATE::INIT:
        message_id = read_data();
        if(message_id==ubx::NAV_SOL)
        {
            int date[6];
            char date_s[128];
            epoch2date(date, ecef_solution.time);
            date2str(date_s, date);
            printf("[%f | ", (double)ecef_solution.time.time+ecef_solution.time.second);
            printf("%s] ECEF:\t", date_s);
            if(ecef_solution.fix_type<sizeof(fix_message))
                printf("<%s> ", fix_message[ecef_solution.fix_type]);
            else
                printf("<Unknown> ");
            printf("Pos= <");
            for(int i=0; i<3; i++)
                printf("%.3f ", ecef_solution.position[i]);
            printf(">  Accuracy= %.3f ", ecef_solution.position_accuracy);
            printf(" pdop = %.2f ", ecef_solution.p_dop);
            printf(" numSV = %i\n", ecef_solution.num_SV);

            
        }
        //if(get_time>T0+Duree)
        //    state = state::RUN;
        break;

    case STATE::RUN:
        break;
    }
}

//-----------------------------------------------------------------------------
int main(void)
{
    if(!configure())
        return -1;

    state = STATE::INIT;
    while(1)
        update();

    return 0;
}
