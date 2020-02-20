#ifndef __TIME_UTIL_HH__
#define __TIME_UTIL_HH__

#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>

namespace time_util
{
/** UTC Time/Julian Day/GPS Time **/
constexpr double SEC_PER_DAY = 86400.0;
constexpr double SEC_PER_WEEK = 604800.0;

/* Difference between TAI and GPS */
constexpr uint32_t TAI_GPS_DIFF = 19;
/* MJD of Unix epoch : 00:00:00 on January 1, 1970 */
constexpr uint32_t MJD_UNIX = 40587;
/* MJD of GPS epoch : Jan. 6 1980 */
constexpr double MJD_JAN61980 = 44244.0;
/* MJD of Jan 1, 1901 */
constexpr double MJD_JAN11901 = 15385.0;

class GPS_TIME;
class UTC_TIME;
class Modified_julian_date;

/* GPS Weeks and Second of Week, NO LEAP SECOND */
class GPS_TIME
{
public:
    GPS_TIME(){};
    explicit GPS_TIME(UTC_TIME);
    explicit GPS_TIME(Modified_julian_date);
    explicit GPS_TIME(time_t);
    virtual ~GPS_TIME();

    GPS_TIME &operator+=(double input);
    GPS_TIME &operator-=(double input);

    GPS_TIME operator+(double input);
    GPS_TIME operator-(double input);

    GPS_TIME &operator-=(GPS_TIME &input);
    GPS_TIME operator-(GPS_TIME &input);

    uint32_t get_week() { return week; }
    double get_SOW() { return SOW; }

    void set_week(uint32_t input) { week = input; }
    void set_SOW(double input) { SOW = input; }

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

    void set_year(uint32_t input) { year = input; }
    void set_month(uint32_t input) { month = input; }
    void set_day(uint32_t input) { day = input; }
    void set_hour(uint32_t input) { hour = input; }
    void set_min(uint32_t input) { min = input; }
    void set_sec(double input) { sec = input; }

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

class Modified_julian_date
{
public:
    Modified_julian_date(){};
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
    double modified_julian_date; /* *io (--) julian date */
};

double time_fmod(double a, double b);

}  // namespace time_util



#endif
