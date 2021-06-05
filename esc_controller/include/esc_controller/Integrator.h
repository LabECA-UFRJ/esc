#include <algorithm>

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

class Integrator
{
public:
    Integrator(double windupLimit)
        : _windupLimit(fabs(windupLimit)), _value(0), _lastInput(0)
    {
    }

    void Update(double dt, double newInput)
    {
        _value += dt * (_lastInput + newInput) * 0.5;
        _value = std::clamp(_value, -_windupLimit, _windupLimit);
        _lastInput = newInput;
    }

    double Value()
    {
        return _value;
    }

private:
    double _lastInput;
    double _value;

    double _windupLimit;
};

#endif