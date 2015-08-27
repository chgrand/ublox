// gps_time.h
#include <string>
#include <time.h>

typedef struct
{
    time_t time;     // time in seconds since epoch (01/01/1970)
    double second;
} gps_time_t;


bool date2epoch(gps_time_t &time, const int date[]);
void epoch2date(int date[], const gps_time_t &time);
void date2str(char *buf, const int date[]);
std::string date2string(const int date[]);

void gps2epoch(gps_time_t &time, int week, double tow);
void galileo2epoch(gps_time_t &time, int week, double tow);
bool glonass2epoch(gps_time_t &time, int week, double tow);
