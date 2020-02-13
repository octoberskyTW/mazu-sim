#ifndef __TIME_UTIL_HH__
#define __TIME_UTIL_HH__

#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>

namespace time_util
{
/** UTC Time/Julian Day/GPS Time **/
const double DJ00 = 2451545.0;
const double DJC = 36525.0;
const double DAS2R = 4.848136811095359935899141e-6;  /* Arcseconds to radians */
const double D2PI = 6.283185307179586476925287;      /* 2Pi */
const double DMAS2R = 4.848136811095359935899141e-9; /* Milliarcseconds to radians */
const double TURNAS = 1296000.0;                     /* Arcseconds in a full circle */
const double SEC_PER_DAY = 86400.0;
const double SEC_PER_WEEK = 604800.0;
const double TT_TAI = 32.184;

/* Difference between TAI and GPS */
const uint32_t TAI_GPS_DIFF = 19;
/* MJD of Unix epoch : 00:00:00 on January 1, 1970 */
const uint32_t MJD_UNIX = 40587;
/* MJD of GPS epoch : Jan. 6 1980 */
const double MJD_JAN61980 = 44244.0;
/* MJD of Jan 1, 1901 */
const double MJD_JAN11901 = 15385.0;

class GPS_TIME;
class UTC_TIME;
class CCSDS_CUC;
class Modified_julian_date;

/* GPS Weeks and Second of Week, NO LEAP SECOND */
class GPS_TIME
{
public:
    GPS_TIME(){};
    explicit GPS_TIME(UTC_TIME);
    explicit GPS_TIME(CCSDS_CUC);
    explicit GPS_TIME(Modified_julian_date);
    explicit GPS_TIME(time_t);
    virtual ~GPS_TIME();

    GPS_TIME &operator+=(double in);
    GPS_TIME &operator-=(double in);

    GPS_TIME operator+(double in);
    GPS_TIME operator-(double in);

    GPS_TIME &operator-=(GPS_TIME &in);
    GPS_TIME operator-(GPS_TIME &in);

    uint32_t get_week() { return week; }
    double get_SOW() { return SOW; }

    void set_week(uint32_t in) { week = in; }
    void set_SOW(double in) { SOW = in; }

    time_t to_unix();

private:
    uint32_t week; /* *io (--) GPS week */
    double SOW;    /* *io (s)  GPS seconds of week */
};

class UTC_TIME
{
public:
    UTC_TIME(){};
    explicit UTC_TIME(GPS_TIME);
    explicit UTC_TIME(Modified_julian_date);
    virtual ~UTC_TIME();

    uint32_t get_year() { return year; }
    uint32_t get_month() { return month; }
    uint32_t get_day() { return day; }
    uint32_t get_hour() { return hour; }
    uint32_t get_min() { return min; }
    double get_sec() { return sec; }

    void set_year(uint32_t in) { year = in; }
    void set_month(uint32_t in) { month = in; }
    void set_day(uint32_t in) { day = in; }
    void set_hour(uint32_t in) { hour = in; }
    void set_min(uint32_t in) { min = in; }
    void set_sec(double in) { sec = in; }

    uint32_t get_day_of_year();
    void set_day_of_year(uint32_t year_in, uint32_t doy);

private:
    uint32_t year;  /* *io (--) Year    */
    uint32_t month; /* *io (--) Month   */
    uint32_t day;   /* *io (--) Day     */
    uint32_t hour;  /* *io (--) Hours.  */
    uint32_t min;   /* *io (--) Minutes */
    double sec;     /* *io (--) Seconds */
};

/* CCSDS Unsegmented time Code */
class CCSDS_CUC
{
public:
    CCSDS_CUC(){};
    explicit CCSDS_CUC(GPS_TIME);
    virtual ~CCSDS_CUC();

    float operator-(CCSDS_CUC &in);

    uint8_t get_C1() { return C1; }
    uint8_t get_C2() { return C2; }
    uint8_t get_C3() { return C3; }
    uint8_t get_C4() { return C4; }
    uint8_t get_F1() { return F1; }
    uint8_t get_F2() { return F2; }

    void set_C1(uint8_t in) { C1 = in; }
    void set_C2(uint8_t in) { C2 = in; }
    void set_C3(uint8_t in) { C3 = in; }
    void set_C4(uint8_t in) { C4 = in; }
    void set_F1(uint8_t in) { F1 = in; }
    void set_F2(uint8_t in) { F2 = in; }

private:
    uint8_t C1; /* *io (--) Coarse time part 1 */
    uint8_t C2; /* *io (--) Coarse time part 2 */
    uint8_t C3; /* *io (--) Coarse time part 3 */
    uint8_t C4; /* *io (--) Coarse time part 4 */
    uint8_t F1; /* *io (--) Fine time part 1 */
    uint8_t F2; /* *io (--) Fine time part 2 */
};

class Modified_julian_date
{
public:
    Modified_julian_date() {}
    explicit Modified_julian_date(GPS_TIME);
    explicit Modified_julian_date(UTC_TIME);
    explicit Modified_julian_date(time_t);
    virtual ~Modified_julian_date();

    void set_mjd(double in) { modified_julian_date = in; }

    double get_mjd() { return modified_julian_date; }
    double get_jd() { return modified_julian_date + 2400000.5; }

    uint32_t tai_leap_second();

    time_t to_unix();

private:
    double modified_julian_date; /* *io (s) julian date */
};

double time_fmod(double a, double b);

}  // namespace time_util



#endif
