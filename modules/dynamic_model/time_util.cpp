#include <exception>
#include "time_util.hh"

time_util::GPS_TIME::GPS_TIME(UTC_TIME in)
{
    double mjd;
    double fmjd;

    /* DOY -> mjd */
    mjd = ((in.get_year() - 1901) / 4) * 1461 + ((in.get_year() - 1901) % 4) * 365 + in.get_day_of_year() - 1 + MJD_JAN11901;
    fmjd = ((in.get_sec() / 60.0 + in.get_min()) / 60.0 + in.get_hour()) / 24.0;

    /* mjd -> gps */
    this->week = static_cast<uint32_t>((mjd - MJD_JAN61980) / 7.0);
    this->SOW = ((mjd - MJD_JAN61980) - this->week * 7 + fmjd) * SEC_PER_DAY;

    uint32_t GPS_UTC_DIFF = Modified_julian_date(in).tai_leap_second() - TAI_GPS_DIFF;

    *this += GPS_UTC_DIFF;
}

time_util::GPS_TIME::GPS_TIME(CCSDS_CUC in)
{
    double integral_part, fractional_part;

    integral_part = in.get_C1() * 16777216.0 + in.get_C2() * 65536.0 + in.get_C3() * 256.0 + in.get_C4();
    fractional_part = in.get_F1() * 0.00390625 + in.get_F2() * 0.0000152587890625;

    /* The difference between GPS epoch (Jan. 6 1980) and the beginning */
    /* of the CUC time (Jan. 6 1981) is 52 weeks + 2 days (172800 secs) */
    /* Note: 1980 is a leap year */
    this->week = static_cast<uint32_t>(floor(integral_part / SEC_PER_WEEK)) + 52;
    this->SOW = time_fmod(integral_part, SEC_PER_WEEK) + fractional_part + 172800.0;
    if (this->SOW >= SEC_PER_WEEK) {
        this->week = static_cast<uint32_t>(this->week + floor(this->SOW / SEC_PER_WEEK));
        this->SOW = time_fmod(this->SOW, SEC_PER_WEEK);
    }
}

time_util::GPS_TIME::GPS_TIME(Modified_julian_date in)
{
    double imjd = floor(in.get_mjd());
    double fmjd = in.get_mjd() - floor(in.get_mjd());
    uint32_t GPS_UTC_DIFF = in.tai_leap_second() - TAI_GPS_DIFF;

    this->week = static_cast<uint32_t>((imjd - MJD_JAN61980) / 7.0);
    this->SOW = ((imjd - MJD_JAN61980) - this->week * 7 + fmjd) * SEC_PER_DAY;

    *this += GPS_UTC_DIFF;
}

time_util::GPS_TIME::GPS_TIME(time_t in) : time_util::GPS_TIME(Modified_julian_date(in))
{
}

time_util::GPS_TIME::~GPS_TIME()
{
}

time_util::GPS_TIME &time_util::GPS_TIME::operator+=(double in)
{
    double temp = this->SOW + in;

    if (fabs(in) > SEC_PER_WEEK) {
        throw std::logic_error("Sorry, Can't increment time by >= 1 week\n");
    }

    if (temp < 0.0) {
        this->week = this->week - 1;
        this->SOW = SEC_PER_WEEK + temp;
    } else if (temp >= SEC_PER_WEEK) {
        this->week = this->week + 1;
        this->SOW = temp - SEC_PER_WEEK;
    } else {
        this->SOW = temp;
    }

    return *this;
}

time_util::GPS_TIME &time_util::GPS_TIME::operator-=(double in)
{
    return this->operator+=(-in);
}

time_util::GPS_TIME time_util::GPS_TIME::operator+(double in)
{
    return time_util::GPS_TIME(*this) += in;
}

time_util::GPS_TIME time_util::GPS_TIME::operator-(double in)
{
    return time_util::GPS_TIME(*this) -= in;
}

time_util::GPS_TIME &time_util::GPS_TIME::operator-=(time_util::GPS_TIME &in)
{
    /* substracts the week number */
    this->week -= in.week;

    /* substracts the number of elapsed second since the begining of the */
    /* current week and the number of 1/256th of second since last second */
    this->SOW -= in.SOW;

    /* updates the week number if the number of elapsed second since the */
    /* beginning of the current week is less than 0 */
    if (this->SOW < 0.0) {
        this->SOW += SEC_PER_WEEK;
        this->week = this->week - 1;
    }

    return *this;
}

time_util::GPS_TIME time_util::GPS_TIME::operator-(time_util::GPS_TIME &in)
{
    return time_util::GPS_TIME(*this) -= in;
}

time_t time_util::GPS_TIME::to_unix()
{
    return Modified_julian_date(*this).to_unix();
}

time_util::UTC_TIME::UTC_TIME(GPS_TIME in)
{
    double mjd, fmjd, days_since_jan1_1901;
    int delta_yrs, num_four_yrs, years_so_far, days_left;

    /* Convert GPS time to MJD */
    // XXX: Fraction of error between mjd and fmjd calculated here and Modified Julian Date Class
    //      Which is correct?
    Modified_julian_date mjd_in(in);
    int32_t UTC_GPS_DIFF = TAI_GPS_DIFF - mjd_in.tai_leap_second();
    in += UTC_GPS_DIFF;
    mjd = in.get_week() * 7.0 + floor(in.get_SOW() / SEC_PER_DAY) + MJD_JAN61980;
    fmjd = time_fmod(in.get_SOW(), SEC_PER_DAY) / SEC_PER_DAY;

    days_since_jan1_1901 = mjd - MJD_JAN11901;
    num_four_yrs = static_cast<int32_t>(days_since_jan1_1901 / 1461); /* 4 years = 1461 days */
    years_so_far = 1901 + 4 * num_four_yrs;
    days_left = static_cast<int64_t>(days_since_jan1_1901 - 1461 * num_four_yrs);
    delta_yrs = days_left / 365 - days_left / 1460;

    uint32_t year = static_cast<uint32_t>(years_so_far + delta_yrs);
    uint32_t DOY = static_cast<uint32_t>(days_left - 365 * delta_yrs + 1);
    this->set_day_of_year(year, DOY);
    this->hour = static_cast<int32_t>(fmjd * 24.0);
    this->min = static_cast<int32_t>(fmjd * 1440.0 - this->hour * 60.0);
    this->sec = fmjd * 86400.0 - this->hour * 3600.0 - this->min * 60.0;
}

/* Create a CalDate using MJD. From Montenbruck C++ code. */
/* Astronomy on the Personal Computer (Springer, ISBN: 0387577009) */
time_util::UTC_TIME::UTC_TIME(Modified_julian_date in)
{
    double mjd = in.get_mjd();

    uint64_t a, b, c, d, e, f, temp;
    double Hours, x;

    a = static_cast<int64_t>(mjd + 2400001.0);

    if (a < 2299161) {
        c = a + 1524;
    } else {
        b = static_cast<int64_t>((a - 1867216.25) / 36524.25);
        c = a + b - (b / 4) + 1525;
    }

    d = static_cast<int64_t>((c - 122.1) / 365.25);
    e = 365 * d + d / 4;
    f = static_cast<int64_t>((c - e) / 30.6001);

    temp = static_cast<int64_t>(30.6001 * f);
    this->day = static_cast<int32_t>(c - e - temp);

    temp = static_cast<int64_t>(f / 14);
    this->month = static_cast<int32_t>(f - 1 - 12 * temp);

    temp = static_cast<int64_t>((7 + this->get_month()) / 10);
    this->year = static_cast<int32_t>(d - 4715 - temp);

    Hours = 24.0 * (mjd - floor(mjd));
    this->hour = static_cast<int32_t>(Hours);

    x = (Hours - this->get_hour()) * 60.0;
    this->min = static_cast<int32_t>(x);
    this->sec = (x - this->get_min()) * 60.0;
}

time_util::UTC_TIME::~UTC_TIME()
{
}


/* Convert YY-MM-DD to Day of Year */
uint32_t time_util::UTC_TIME::get_day_of_year()
{
    unsigned int regu_month_day[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
    unsigned int leap_month_day[12] = { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 };
    unsigned int yday = 0;

    /* check for leap year */
    if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) {
        yday = leap_month_day[month - 1] + day;
    } else {
        yday = regu_month_day[month - 1] + day;
    }

    return yday;
}

/* Setting UTC_TIME with Day of year */
void time_util::UTC_TIME::set_day_of_year(uint32_t year_in, uint32_t doy)
{
    unsigned int month_array[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    unsigned int month_array_leap_year[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    unsigned int month = 0;

    /* check for leap year */
    if ((year_in % 4 == 0 && year_in % 100 != 0) || year_in % 400 == 0) {
        while (doy > month_array_leap_year[month]) {
            doy = doy - month_array_leap_year[month];
            month = month + 1;
        }
    } else {
        while (doy > month_array[month]) {
            doy = doy - month_array[month];
            month = month + 1;
        }
    }

    this->year = year_in;
    this->month = month + 1;
    this->day = doy;
}

time_util::CCSDS_CUC::CCSDS_CUC(GPS_TIME in)
{
    double total_sec, C1_sec, C2_sec, C3_sec;

    /* GPS epoch : Jan. 6 1980 */
    /* CUC epoch : Jan. 6 1981 */
    /* Difference between CUC and GPS is 52 weeks and 2 days (172800 secs) */
    if (((in.get_week() == 52.0) && (in.get_SOW() >= 172800.0)) ||
        (in.get_week() > 52.0)) {
        if (in.get_SOW() < 172800.0) {
            total_sec = (in.get_week() - 53.0) * SEC_PER_WEEK + (SEC_PER_WEEK - 172800.0) + in.get_SOW();
        } else {
            total_sec = (in.get_week() - 52.0) * SEC_PER_WEEK + (in.get_SOW() - 172800.0);
        }

        this->C1 = (unsigned char) (total_sec / 16777216.0);
        C1_sec = this->C1 * 16777216.0; /* 256^3 */

        this->C2 = (unsigned char) ((total_sec - C1_sec) / 65536.0);
        C2_sec = this->C2 * 65536.0; /* 256^2 */

        this->C3 = (unsigned char) ((total_sec - C1_sec - C2_sec) / 256.0);
        C3_sec = this->C3 * 256.0;

        this->C4 = (unsigned char) (floor(total_sec - C1_sec - C2_sec - C3_sec));

        total_sec = total_sec - C1_sec - C2_sec - C3_sec - this->C4;

        this->F1 = (unsigned char) (total_sec / 0.00390625);
        this->F2 = (unsigned char) ((total_sec - this->F1 * 0.00390625) / 0.0000152587890625);
    } else {
        throw std::out_of_range("CUC time is not available!");
    }
}

time_util::CCSDS_CUC::~CCSDS_CUC() {}

float time_util::CCSDS_CUC::operator-(time_util::CCSDS_CUC &in)
{
    float diff;

    diff = static_cast<float>((this->C1 - in.C1) * 16777216.0                                                     /* 256^3 */
                              + (this->C2 - in.C2) * 65536.0                                                      /* 256^2 */
                              + (this->C3 - in.C3) * 256.0 + (this->C4 - in.C4) + (this->F1 - in.F1) * 0.00390625 /* 256^(-1) */
                              + (this->F2 - in.F2) * 0.0000152587890625);                                         /*256^(-2) */

    return diff;
}

time_util::Modified_julian_date::Modified_julian_date(GPS_TIME in)
{
    this->modified_julian_date = in.get_week() * 7.0 + in.get_SOW() / SEC_PER_DAY + MJD_JAN61980; /* 44244 */

    int32_t UTC_GPS_DIFF = TAI_GPS_DIFF - this->tai_leap_second();

    in += UTC_GPS_DIFF;

    this->modified_julian_date = in.get_week() * 7.0 + in.get_SOW() / SEC_PER_DAY + MJD_JAN61980; /* 44244 */
}

time_util::Modified_julian_date::Modified_julian_date(UTC_TIME in)
{
    int MjdMidnight;
    double FracOfDay, mjd;
    int b, temp;

    /*double jd;*/
    if (in.get_month() <= 2) {
        in.set_month(in.get_month() + 12);
        in.set_year(in.get_year() - 1);
    }

    if ((10000 * in.get_year() + 100 * in.get_month() + in.get_day()) <= 15821004) {
        /* For a date in the Julian calendar , up to 1582 October 4 */
        b = -2 + static_cast<int32_t>((in.get_year() + 4716) / 4) - 1179;
    } else {
        /* Gregorian calendar is used from 1582 October 15 onwards */
        b = static_cast<int32_t>(in.get_year() / 400) - static_cast<int32_t>(in.get_year() / 100) + static_cast<int32_t>(in.get_year() / 4);
    }

    temp = static_cast<int32_t>(30.6001 * (in.get_month() + 1));

    MjdMidnight = 365 * in.get_year() - 679004 + b + temp + in.get_day();
    /*FracOfDay   = (in.get_hour() + in.get_min()/60.0 + in.get_sec()/3600.0) / 24.0; */
    FracOfDay = (in.get_hour() * 3600 + in.get_min() * 60.0 + in.get_sec()) / SEC_PER_DAY;

    mjd = static_cast<double>(MjdMidnight) + FracOfDay;

    /*printf("mjd = %.15f\n", mjd); */

    /*** Algorithm provided by Vallado's book 3rd Edition ***/
    /*** page 189, Algorithm 14, verified same result as  ***/
    /*** the above algorithm.                             ***/
    /***
    double jd;

    jd = 367.0 * in.get_year() -
         floor((7 * (in.get_year() + floor((in.get_month() + 9) / 12.0))) * 0.25) +
         floor( 275 * in.get_month() / 9.0 ) +
         in.get_day() + 1721013.5 +
         ((in.get_sec() / 60.0 + in.get_min()) / 60.0 + in.get_hour()) / 24.0; ***/
    /* - 0.5 * sgn(100.0*year + mon - 190002.5) + 0.5; */

    /***
    mjd = jd - 2400000.5;
    ***/

    this->modified_julian_date = mjd;
}

time_util::Modified_julian_date::Modified_julian_date(time_t in)
{
    this->modified_julian_date = in / SEC_PER_DAY + MJD_UNIX;
}

time_util::Modified_julian_date::~Modified_julian_date()
{
}

// Return the difference between TAI and UTC (known as leap seconds).
// Values from the USNO website: ftp://maia.usno.navy.mil/ser7/leapsec.dat
//                               ftp://maia.usno.navy.mil/ser7/tai-utc.dat
// Check IERS Bulletin C. (International Earth Rotation and Reference
// Systems Service) http://www.iers.org/
// @param mjd Modified Julian Date
// @return number of leaps seconds.
uint32_t time_util::Modified_julian_date::tai_leap_second()
{
    double mjd = this->modified_julian_date;

    if (mjd < 0.0) {
        throw std::out_of_range("MJD before the beginning of the leap sec table");
        return 0;
    }

    if ((mjd >= 41317.0) && (mjd < 41499.0))
        return 10; /* January 1, 1972 */
    if ((mjd >= 41499.0) && (mjd < 41683.0))
        return 11; /* July 1, 1972    */
    if ((mjd >= 41683.0) && (mjd < 42048.0))
        return 12; /* January 1, 1973 */
    if ((mjd >= 42048.0) && (mjd < 42413.0))
        return 13; /* January 1, 1974 */
    if ((mjd >= 42413.0) && (mjd < 42778.0))
        return 14; /* January 1, 1975 */
    if ((mjd >= 42778.0) && (mjd < 43144.0))
        return 15; /* January 1, 1976 */
    if ((mjd >= 43144.0) && (mjd < 43509.0))
        return 16; /* January 1, 1977 */
    if ((mjd >= 43509.0) && (mjd < 43874.0))
        return 17; /* January 1, 1978 */
    if ((mjd >= 43874.0) && (mjd < 44239.0))
        return 18; /* January 1, 1979 */
    if ((mjd >= 44239.0) && (mjd < 44786.0))
        return 19; /* January 1, 1980 */
    if ((mjd >= 44786.0) && (mjd < 45151.0))
        return 20; /* July 1, 1981    */
    if ((mjd >= 45151.0) && (mjd < 45516.0))
        return 21; /* July 1, 1982    */
    if ((mjd >= 45516.0) && (mjd < 46247.0))
        return 22; /* July 1, 1983    */
    if ((mjd >= 46247.0) && (mjd < 47161.0))
        return 23; /* July 1, 1985    */
    if ((mjd >= 47161.0) && (mjd < 47892.0))
        return 24; /* January 1, 1988 */
    if ((mjd >= 47892.0) && (mjd < 48257.0))
        return 25; /* January 1, 1990 */
    if ((mjd >= 48257.0) && (mjd < 48804.0))
        return 26; /* January 1, 1991 */
    if ((mjd >= 48804.0) && (mjd < 49169.0))
        return 27; /* July 1st, 1992  */
    if ((mjd >= 49169.0) && (mjd < 49534.0))
        return 28; /* July 1, 1993    */
    if ((mjd >= 49534.0) && (mjd < 50083.0))
        return 29; /* July 1, 1994    */
    if ((mjd >= 50083.0) && (mjd < 50630.0))
        return 30; /* January 1, 1996 */
    if ((mjd >= 50630.0) && (mjd < 51179.0))
        return 31; /* July 1, 1997    */
    if ((mjd >= 51179.0) && (mjd < 53736.0))
        return 32; /* January 1, 1999 */
    if ((mjd >= 53736.0) && (mjd < 54832.0))
        return 33; /* January 1, 2006 */
    if ((mjd >= 54832.0) && (mjd < 56109.0))
        return 34; /* January 1, 2009 */
    if ((mjd >= 56109.0) && (mjd < 57204.0))
        return 35; /* July 1, 2012 */
    if ((mjd >= 57204.0) && (mjd < 57754.0))
        return 36; /* July 1, 2015 */
    if (mjd >= 57754.0)
        return 37; /* January 1, 2017 */

    throw std::out_of_range("Input MJD out of bounds");
    return 0;
}

time_t time_util::Modified_julian_date::to_unix()
{
    return static_cast<int64_t>((this->modified_julian_date - MJD_UNIX) * SEC_PER_DAY);
}

double time_util::time_fmod(double a, double b)
{
    double temp, absu;

    absu = static_cast<double>(fabs(a));
    temp = absu - b * static_cast<int32_t>(absu / b);
    if (temp < 0.0)
        temp = temp + fabs(b);
    if (a < 0.0)
        temp = -temp;
    return temp;
}
