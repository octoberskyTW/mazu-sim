#include "schmitt_trigger.hh"

#include "matrix_tool.hh"

Schmitt_Trigger::Schmitt_Trigger(double dead_zone, double hysteresis)
    : dead_zone(dead_zone), hysteresis(hysteresis)
{
    this->saved_value = 0;
}

Schmitt_Trigger::Schmitt_Trigger(const Schmitt_Trigger &other)
{
    this->saved_value = other.saved_value;
    this->dead_zone = other.dead_zone;
    this->hysteresis = other.hysteresis;
}

Schmitt_Trigger &Schmitt_Trigger::operator=(const Schmitt_Trigger &other)
{
    if (&other == this)
        return *this;

    this->saved_value = other.saved_value;
    this->dead_zone = other.dead_zone;
    this->hysteresis = other.hysteresis;

    return *this;
}

int Schmitt_Trigger::trigger(double in)
{
    // local variable
    int output(0);

    // saved_value signal 'trend' (=1 increasing, =-1 decreasing)
    // saved_value signal 'side' (=-1 left, =1 right)
    int trend = sign(in - saved_value);
    int side = sign(saved_value);
    double trigger = (dead_zone * side + hysteresis * trend) / 2;

    if (saved_value >= trigger && side == 1) {
        output = 1;
    } else if (saved_value <= trigger && side == -1) {
        output = -1;
    } else {
        output = 0;
    }

    saved_value = in;

    return output;
}

void Schmitt_Trigger::clear()
{
    this->saved_value = 0;

    return;
}
