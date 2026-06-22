#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <cmath>

class Chebyshev2Filter {
private:
    const float b0 = 0.04125354f;
    const float b1 = 0.08250707f;
    const float b2 = 0.04125354f;
    const float a1 = -1.34896775f;
    const float a2 = 0.51398189f;
    const float gain = 1.0f;

    float x[3] = {0.0f, 0.0f, 0.0f};
    float y[3] = {0.0f, 0.0f, 0.0f};
    bool first = true;

public:
    Chebyshev2Filter() = default;

    float filter(float input)
    {
        if (first)
        {
            x[2] = input; x[1] = input; x[0] = input;
            y[2] = input; y[1] = input; y[0] = input;
            first = false;
        } 
        else
        {
            x[2] = x[1];
            x[1] = x[0];
            x[0] = input;
            
            y[2] = y[1];
            y[1] = y[0];
            
            y[0] = gain * (b0*x[0] + b1*x[1] + b2*x[2] - (a1 * y[1] + a2 * y[2]));
        }

        if (std::isinf(y[0]) || std::isnan(y[0])) {
            first = true; // Force reset on next execution
            return input; // Fallback safely to raw input
        }

        return y[0];
    }

    void reset()
    {
        first = true;
    }
};

#endif