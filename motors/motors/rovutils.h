#ifndef _ROVUTILS_H_
#define _ROVUTILS_H_
#include <math.h>

namespace rovUtils{
    static unsigned short sign = 1;
    double seaLevel(double P, double A)
    // Given a pressure P (mbar) taken at a specific altitude (meters),
    // return the equivalent pressure (mbar) at sea level.
    // This produces pressure readings that can be used for weather measurements.
    {
        return(P/pow(1-(A/44330.0),5.255));
    }


    double altitude(double P, double P0)
    // Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
    // return altitude (meters) above baseline.
    {
        return(44330.0*(1-pow(P/P0,1/5.255)));
    }

    int rampMotor(int current_value, int end_value, int step_value)
    {
        
        if(end_value < current_value)
        {
            sign = -1;
        } else {
            sign = 1;
        }
        int new_speed = current_value + step_value*sign;
        if(end_value*sign < new_speed*sign)
        {
            return end_value;
        } else {
            return new_speed;
        }
        return current_value;
    }

}




#endif _ROVUTILS_H_