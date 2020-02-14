#ifndef __SIGNAL_PROCESS_HH__
#define __SIGNAL_PROCESS_HH__
#include <cmath>
#include <cstdlib>
#include <iostream>

class FilterType
{
public:
    FilterType(){};
    ~FilterType(){};
    virtual double Process(double x_in) = 0;

protected:
    long Ns;
    double *A, *B, *x, *y;
    double dxmax;
    double ymin;
};

class FirstOrderLowpassFilter : public FilterType
{
public:
    explicit FirstOrderLowpassFilter(double w, double T,
                                     double dxmax_in, double ymin_in);
    ~FirstOrderLowpassFilter();
    virtual double Process(double x_in);
};

class FirstOrderHighpassFilter : public FilterType
{
public:
    explicit FirstOrderHighpassFilter(double w, double T,
                                      double dxmax_in, double ymin_in);
    ~FirstOrderHighpassFilter();
    virtual double Process(double x_in);
};

class SecondOrderLowpassFilter : public FilterType
{
public:
    explicit SecondOrderLowpassFilter(double w, double T, double z,
                                      double dxmax_in, double ymin_in);
    ~SecondOrderLowpassFilter();
    virtual double Process(double x_in);
};

class SecondOrderHighpassFilter : public FilterType
{
public:
    explicit SecondOrderHighpassFilter(double w, double T, double z,
                                       double dxmax_in, double ymin_in);
    ~SecondOrderHighpassFilter();
    virtual double Process(double x_in);
};
#endif  //  __SIGNAL_PROCESS_HH__