#include "signal_process.hh"

FirstOrderLowpassFilter::FirstOrderLowpassFilter(double w, double T,
                                                 double dxmax_in,
                                                 double ymin_in)
{
    if (6.28 / w < 10.0 * T) {
        std::cout << "Error in Create 1st Order LowpassFilter." << std::endl;
        std::cout << "Filter break frequency is too high to be sampled at given "
                     "sample rate."
                  << std::endl;
        std::cout << "Recommend sample interval be reduced to" << (0.628 / w)
                  << " sec or less." << std::endl;
        exit(1);
    }

    Ns = 2;
    A = (double *) calloc(Ns, sizeof(double));
    B = (double *) calloc(Ns, sizeof(double));
    x = (double *) calloc(Ns, sizeof(double));
    y = (double *) calloc(Ns, sizeof(double));
    A[0] = exp(-w * T);
    B[0] = 0.5 * (1.0 - A[0]);
    dxmax = dxmax_in;
    ymin = ymin_in;

    return;
}

FirstOrderLowpassFilter::~FirstOrderLowpassFilter()
{
    delete[] A;
    delete[] B;
    delete[] x;
    delete[] y;
}

double FirstOrderLowpassFilter::Process(double x_in)
{
    double Xmax, Xmin;
    long k;

    /* Clamp spikes */
    x[0] = x_in;
    Xmax = x[1] + dxmax;
    if (x[0] > Xmax)
        x[0] = Xmax;
    Xmin = x[1] - dxmax;
    if (x[0] < Xmin)
        x[0] = Xmin;

    /* Filter */
    y[0] = A[0] * y[1] + B[0] * (x[0] + x[1]);

    /* Trap underflow */
    if (fabs(y[0]) < ymin)
        y[0] = 0.0;

    for (k = Ns - 1; k > 0; k--) {
        x[k] = x[k - 1];
        y[k] = y[k - 1];
    }

    return (y[0]);
}

FirstOrderHighpassFilter::FirstOrderHighpassFilter(double w, double T,
                                                   double dxmax_in,
                                                   double ymin_in)
{
    if (6.28 / w < 10.0 * T) {
        std::cout << "Error in Create 1st Order HighpassFilter." << std::endl;
        std::cout << "Filter break frequency is too high to be sampled at given "
                     "sample rate."
                  << std::endl;
        std::cout << "Recommend sample interval be reduced to" << (0.628 / w)
                  << " sec or less." << std::endl;
        exit(1);
    }

    Ns = 2;
    A = (double *) calloc(Ns, sizeof(double));
    B = (double *) calloc(Ns, sizeof(double));
    x = (double *) calloc(Ns, sizeof(double));
    y = (double *) calloc(Ns, sizeof(double));
    A[0] = exp(-w * T);
    B[0] = 0.5 * (1.0 + A[0]);
    dxmax = dxmax_in;
    ymin = ymin_in;

    return;
}

double FirstOrderHighpassFilter::Process(double x_in)
{
    double Xmax, Xmin;
    long k;

    /* Clamp spikes */
    x[0] = x_in;
    Xmax = x[1] + dxmax;
    if (x[0] > Xmax)
        x[0] = Xmax;
    Xmin = x[1] - dxmax;
    if (x[0] < Xmin)
        x[0] = Xmin;

    /* Filter */
    y[0] = A[0] * y[1] + B[0] * (x[0] - x[1]);

    /* Trap underflow */
    if (fabs(y[0]) < ymin)
        y[0] = 0.0;

    for (k = Ns - 1; k > 0; k--) {
        x[k] = x[k - 1];
        y[k] = y[k - 1];
    }

    return (y[0]);
}

FirstOrderHighpassFilter::~FirstOrderHighpassFilter()
{
    delete[] A;
    delete[] B;
    delete[] x;
    delete[] y;
}

SecondOrderLowpassFilter::SecondOrderLowpassFilter(double w, double T, double z,
                                                   double dxmax_in,
                                                   double ymin_in)
{
    double a;
    double unuseda __attribute__((unused)) = ymin_in;
    double unusedb __attribute__((unused)) = dxmax_in;
    if (6.28 / w < 10.0 * T) {
        std::cout << "Error in Create 2nd Order LowpassFilter." << std::endl;
        std::cout << "Filter break frequency is too high to be sampled at given "
                     "sample rate."
                  << std::endl;
        std::cout << "Recommend sample interval be reduced to" << 0.628 / w
                  << "sec or less." << std::endl;
        exit(1);
    }

    Ns = 3;
    A = (double *) calloc(Ns, sizeof(double));
    B = (double *) calloc(Ns, sizeof(double));
    x = (double *) calloc(Ns, sizeof(double));
    y = (double *) calloc(Ns, sizeof(double));

    a = exp(-z * w * T);
    A[0] = 2.0 * a * cos(w * T);
    A[1] = -a * a;
    B[0] = 0.25 * (1.0 - 2.0 * a * cos(w * T) + a * a);
    dxmax = dxmax;
    ymin = ymin;

    return;
}

double SecondOrderLowpassFilter::Process(double x_in)
{
    double Xmax, Xmin;
    long k;

    /* Clamp spikes */
    x[0] = x_in;
    Xmax = x[1] + dxmax;
    if (x[0] > Xmax)
        x[0] = Xmax;
    Xmin = x[1] - dxmax;
    if (x[0] < Xmin)
        x[0] = Xmin;

    /* Filter */
    y[0] = A[0] * y[1] + A[1] * y[2] + B[0] * (x[0] + 2.0 * x[1] + x[2]);

    /* Trap underflow */
    if (fabs(y[0]) < ymin)
        y[0] = 0.0;

    for (k = Ns - 1; k > 0; k--) {
        x[k] = x[k - 1];
        y[k] = y[k - 1];
    }

    return (y[0]);
}

SecondOrderLowpassFilter::~SecondOrderLowpassFilter()
{
    delete[] A;
    delete[] B;
    delete[] x;
    delete[] y;
}

SecondOrderHighpassFilter::SecondOrderHighpassFilter(double w, double T,
                                                     double z, double dxmax_in,
                                                     double ymin_in)
{
    double a;
    double unuseda __attribute__((unused)) = ymin_in;
    double unusedb __attribute__((unused)) = dxmax_in;
    if (6.28 / w < 10.0 * T) {
        std::cout << "Error in Create 2nd Order HighpassFilter." << std::endl;
        std::cout << "Filter break frequency is too high to be sampled at given "
                     "sample rate."
                  << std::endl;
        std::cout << "Recommend sample interval be reduced to" << 0.628 / w
                  << "sec or less." << std::endl;
        exit(1);
    }

    Ns = 3;
    A = (double *) calloc(Ns, sizeof(double));
    B = (double *) calloc(Ns, sizeof(double));
    x = (double *) calloc(Ns, sizeof(double));
    y = (double *) calloc(Ns, sizeof(double));

    a = exp(-z * w * T);
    A[0] = 2.0 * a * cos(w * T);
    A[1] = -a * a;
    B[0] = 0.25 * (1.0 + a * cos(w * T));
    dxmax = dxmax;
    ymin = ymin;

    return;
}

double SecondOrderHighpassFilter::Process(double x_in)
{
    double Xmax, Xmin;
    long k;

    /* Clamp spikes */
    x[0] = x_in;
    Xmax = x[1] + dxmax;
    if (x[0] > Xmax)
        x[0] = Xmax;
    Xmin = x[1] - dxmax;
    if (x[0] < Xmin)
        x[0] = Xmin;

    /* Filter */
    y[0] = A[0] * y[1] + A[1] * y[2] + B[0] * (x[0] - 2.0 * x[1] + x[2]);

    /* Trap underflow */
    if (fabs(y[0]) < ymin)
        y[0] = 0.0;

    for (k = Ns - 1; k > 0; k--) {
        x[k] = x[k - 1];
        y[k] = y[k - 1];
    }

    return (y[0]);
}

SecondOrderHighpassFilter::~SecondOrderHighpassFilter()
{
    delete[] A;
    delete[] B;
    delete[] x;
    delete[] y;
}