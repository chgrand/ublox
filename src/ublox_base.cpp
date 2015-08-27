#include <iostream>
#include <string>
using namespace std;


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
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


const static double D2R = M_PI/180.;
const static double R2D = 180./M_PI;

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

typedef struct
{
    gps_time_t time;  // GPS time
    uint8_t numSV;    // Number of satellite
    uint8_t gnssID;   // Gnss identifier { }
    uint8_t svID;     // space vehicle id
    uint8_t cnor;     // SV Carrier to noise ration (signal strength)
    double elevation; // SV position
    double azimut;    // ""
    double prRes;     // Pseudo range residual
    uint32_t flags;
} gnss_obs_t;


ecef_solution_t ecef_solution;
gnss_obs_t observations;
double ecef_mean_position[3];
int week;


UBX_Port ublox;



//-----------------------------------------------------------------------------
void print_sat(const gnss_obs_t *observations)
{
    string gnss_type[] = { "GPS", "SBAS", "Galileo", "BeiDou",
                           "IMES", "QZSS", "GLONASS"};

    double time = double(observations->time.time+observations->time.second);
    int date[6];
    epoch2date(date, observations->time);

    cout << "Ephemerid of " << observations->numSV
         << " - GPS time " << time
         << " <" << date2string(date) << ">" << endl;

    for(int i=0;i<observations->numSV; i++)
    {
        cout << "* SV = " << gnss_type[observations->gnssID]
             << "-" << observations->svID
             << "\t p=" << observations->cnor << "dBHz"
             << " elev=" << observations->elevation
             << " azim=" << observations->azimut
             << " residual=" << observations->prRes
             << endl;
    }
}

void set_CFG_MSG(uint16_t id, uint8_t cycle)
{
    ubx_packet_t packet;
    packet.id    = ubx::CFG_MSG;
    packet.len  = 3;
    ubx::U2_write(packet.data, id);  // Position in ECEF
    packet.data[2] = cycle;          // At each cycle
    ublox.write_message(&packet);
}

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
    ubx::U2_write(packet.data+0, 500); // Gnss measurement rate in ms
    ubx::U2_write(packet.data+2, 1);   // Nav. Rate, in number of meas. cycles
    ubx::U2_write(packet.data+4, 1);   // Ref. time (0: UTC time, 1: GPS time)
    ublox.write_message(&packet);

    // Set GNSS Nav properties
    packet.id    = ubx::CFG_NAV5;
    ublox.poll_message(&packet);
    packet.data[2] = 2;              // dynModel = 2 (stationnary)
    ublox.write_message(&packet);

    printf("Poll NAV_SAT =============\n");
    packet.id    = ubx::RXM_RAWX;
    packet.len = 0;
    ublox.poll_message(&packet);

/*
    // Select active UBX_MSG
    set_CFG_MSG(ubx::NAV_SOL, 1);
    set_CFG_MSG(ubx::NAV_POSECEF, 1);
    set_CFG_MSG(ubx::NAV_POSLLH, 1);
    set_CFG_MSG(ubx::MGA_GPS, 1);
*/

    ublox.set_debug(false);

    return true;

}

//-----------------------------------------------------------------------------
int read_data()
{
    ubx_packet_t packet;
    double iTow;
    double fTow;
    int version;

    if(!ublox.wait_sync_timeout(10)) // 10ms timeout
        return -1;

    if(!ublox.read_packet(&packet))
        return -1;

    printf("Recv packet with id = %04x ", packet.id);
    switch(packet.id) {
    case ubx::MGA_GPS:
        printf("MGA_GPS\n");
        break;
    case ubx::NAV_POSECEF:
        printf("NAV_POSECEF\n");
/*
        printf("[%i ms]: NAV_POSECEF = ", ubx::U4_read(packet.data));
        printf("%.3f ", (double)ubx::I4_read(packet.data+4)/100.);
        printf("%.3f ", (double)ubx::I4_read(packet.data+8)/100.);
        printf("%.3f ", (double)ubx::I4_read(packet.data+12)/100.);
        printf("(%.3f)\n", (double)ubx::I4_read(packet.data+16)/100.);
*/
        break;

    case ubx::NAV_POSLLH:
        printf("NAV_POSLLH\n");
        /*
        printf("[%i ms]: NAV_POSLLH = ", ubx::U4_read(packet.data));
        printf("%.8f ", (double)ubx::I4_read(packet.data+4)*1e-7);
        printf("%.8f ", (double)ubx::I4_read(packet.data+8)*1e-7);
        printf("%.4f ", (double)ubx::I4_read(packet.data+12)/1000.);
        printf("(%.4f)\n", (double)ubx::I4_read(packet.data+20)/1000.);
        */
        break;

    case ubx::NAV_SOL:
        printf("NAV_SOL\n");
        iTow = (double)ubx::U4_read(packet.data+0)*1e-3;
        fTow = (double)ubx::I4_read(packet.data+4)*1e-9;
        week = int(ubx::I2_read(packet.data+8));
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

    case ubx::NAV_SAT: // get ephemerid data
        printf("NAV_SAT\n");
        {
        iTow = (double)ubx::U4_read(packet.data+0)*1e-3;
        if(ubx::U1_read(packet.data+4)!=1) // wrong message version
            break;
        gps2epoch(ecef_solution.time, week, iTow+fTow);
        observations.numSV = ubx::U1_read(packet.data+5);
        for(int i=0; i <observations.numSV; i++)
        {
            observations.gnssID = ubx::U1_read(packet.data+8+12*i);
            observations.svID   = ubx::U1_read(packet.data+9+12*i);
            observations.cnor   = ubx::U1_read(packet.data+10+12*i);
            observations.elevation = double(ubx::I1_read(packet.data+11+12*i))*D2R;
            observations.azimut = double(ubx::I2_read(packet.data+12+12*i))*D2R;
            observations.prRes = double(ubx::I2_read(packet.data+14+12*i))*0.1;
            observations.flags = ubx::U4_read(packet.data+16+12*i);
        }
    }
    break;

    default:
        printf("Unknown id = %0x4\n", packet.id);
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
            /*
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
            */

            /*
            for(int i=0; i<3; i++)
                printf("%.3f ", ecef_solution.position[i]);

            printf("%.3f", ecef_solution.position_accuracy);
            std::cout << std::endl;
            */
        }
        //if(get_time>T0+Duree)
        //    state = state::RUN;

        if(message_id==ubx::NAV_SAT)
            print_sat(&observations);
        break;


    case STATE::RUN:
        break;
    }
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    if(!configure())
        return -1;

    state = STATE::INIT;
    while(1)
        update();

    return 0;
}
