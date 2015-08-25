#include <math.h>
#include <stdio.h>
#include "gps_time.h"

const static int gps_t0[]     = {1980,1, 6,0,0,0}; /* gps time reference */
const static int galileo_t0[] = {1999,8,22,0,0,0}; /* galileo system time reference */
const static int beidou_t0[]  = {2006,1, 1,0,0,0}; /* beidou time reference */


// Convert date as Year, Month, Day, H, min, sec to time in epoch
bool date2epoch(gps_time_t &time, const int date[])
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    int days,sec,year=(int)date[0],mon=(int)date[1],day=(int)date[2];

    if (year<1970||2099<year||mon<1||12<mon)
        return false;

    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(date[5]);
    time.time=(time_t)days*86400+(int)date[3]*3600+(int)date[4]*60+sec;
    time.second=date[5]-sec;
    return true;
}


void epoch2date(int date[], const gps_time_t &time)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;

    // leap year if year%4==0 in 1901-2099
    days=(int)(time.time/86400);
    sec=(int)(time.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    date[0]=1970+days/1461*4+mon/12; date[1]=mon%12+1; date[2]=day+1;
    date[3]=sec/3600; date[4]=sec%3600/60; date[5]=sec%60+time.second;
}

void date2str(char *buf, const int date[])
{
    sprintf(buf, "% 4i-%02i-%02i % 2i:%02i:%02i",
            date[0], date[1], date[2], date[3], date[4], date[5]);
}

void gps2epoch(gps_time_t &time, int week, double tow)
{
    date2epoch(time, gps_t0);
    time.time += week*(7*86400) + (int)tow;
    time.second += tow-(int)(tow);
}

void galileo2epoch(gps_time_t &time, int week, double tow)
{
}

bool glonass2epoch(gps_time_t &time, int week, double tow)
{
}

